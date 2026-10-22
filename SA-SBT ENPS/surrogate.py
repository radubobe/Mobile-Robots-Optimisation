"""
Black-box surrogate that predicts test outcome without running Webots.

    python surrogate.py compare    # compare model families (cross-validation)
    python surrogate.py final      # train on training set, evaluate on validation set
    python surrogate.py train      # train and save model to disk
    python surrogate.py regress    # regression on continuous fitness
"""

import csv
import pickle
import re
import sys
import warnings

import numpy as np
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

warnings.filterwarnings("ignore")

ROBOT_RADIUS = 0.037

# n_sharp_turns omitted: all-zero at the road widths used.
GEOM = ["length", "min_radius", "p05_radius", "median_radius", "max_curvature",
        "mean_abs_curv", "total_turning", "net_turning", "sinuosity",
        "curv_std", "max_curv_change"]

FEATURE_NAMES = GEOM + [
    "width", "clearance",
    "min_radius/clr", "p05_radius/clr", "max_curv*clr",
    "mean_curv*clr", "total_turn*clr",
    "is_M1", "is_M2", "is_M3",
]

MODELS = ("Model 1", "Model 2", "Model 3")


def load_features(path):
    roads, current = {}, None
    for line in open(path):
        header = re.search(r"([A-Za-z]+\.\d{4})\.txt", line)
        if header:
            current = header.group(1)
            roads[current] = {}
            continue
        parts = line.split()
        if current and len(parts) == 2:
            try:
                roads[current][parts[0]] = float(parts[1])
            except ValueError:
                pass
    return roads


def build(results_path, features_path):
    # groups = road name; each road appears 12 times (4 widths x 3 models),
    # so grouping prevents geometry leaking from train into test.
    roads = load_features(features_path)
    X, y, groups, meta = [], [], [], []
    for row in csv.DictReader(open(results_path)):
        if not row.get("width") or row["road"] not in roads:
            continue
        geom = roads[row["road"]]
        width = float(row["width"])
        clearance = width / 2.0 - ROBOT_RADIUS

        features = [geom[n] for n in GEOM] + [width, clearance]
        # geometry × clearance interactions: the same curve is harder on a narrower road
        features += [geom["min_radius"] / clearance,
                     geom["p05_radius"] / clearance,
                     geom["max_curvature"] * clearance,
                     geom["mean_abs_curv"] * clearance,
                     geom["total_turning"] * clearance]
        features += [1.0 if row["model"] == m else 0.0 for m in MODELS]

        X.append(features)
        y.append(0 if row["status"] == "SUCCESS" else 1)
        groups.append(row["road"])
        meta.append((row["road"], width, row["model"], row["status"]))
    return np.array(X), np.array(y), np.array(groups), meta


def make_model():
    from sklearn.ensemble import RandomForestClassifier
    return RandomForestClassifier(n_estimators=400, min_samples_leaf=2,
                                  random_state=0, n_jobs=-1)


def savings_table(prob, y):
    print("  %-8s %-22s %-24s %s"
          % ("prag", "simulari rulate", "defecte gasite", "ratate"))
    for threshold in (0.05, 0.10, 0.15, 0.20, 0.30, 0.50):
        run = prob > threshold
        found = y[run].sum()
        print("  %-8.2f %-22s %-24s %d"
              % (threshold,
                 "%d / %d  (%.0f%%)" % (run.sum(), len(y), 100 * run.mean()),
                 "%d / %d  (%.0f%%)" % (found, y.sum(), 100 * found / y.sum()),
                 y.sum() - found))


def per_model(y, pred, meta, label="recall"):
    from sklearn.metrics import recall_score, precision_score
    models = np.array([m[2] for m in meta])
    for name in MODELS:
        s = models == name
        if not s.any() or y[s].sum() == 0:
            continue
        print("    %-9s %3d esecuri   recall=%5.1f%%   precizie=%5.1f%%"
              % (name, y[s].sum(),
                 100 * recall_score(y[s], pred[s], zero_division=0),
                 100 * precision_score(y[s], pred[s], zero_division=0)))


def baseline_predict(y_train, meta_train, meta_test):
    models_tr = np.array([m[2] for m in meta_train])
    models_te = np.array([m[2] for m in meta_test])
    out = np.zeros(len(meta_test))
    for name in MODELS:
        tr = models_tr == name
        if tr.any():
            out[models_te == name] = 1 if y_train[tr].mean() > 0.5 else 0
    return out


def cmd_compare(root):
    from sklearn.ensemble import (RandomForestClassifier,
                                  GradientBoostingClassifier,
                                  HistGradientBoostingClassifier)
    from sklearn.neural_network import MLPClassifier
    from sklearn.linear_model import LogisticRegression
    from sklearn.preprocessing import StandardScaler
    from sklearn.pipeline import make_pipeline
    from sklearn.model_selection import GroupKFold
    from sklearn.metrics import (accuracy_score, precision_score, recall_score,
                                 roc_auc_score)

    X, y, g, meta = build(root + "batch_results_training.csv",
                          root + "features_training.txt")
    models = np.array([m[2] for m in meta])
    hard = models != "Model 1"
    print("set: %d randuri, %d trasaturi, %d drumuri, %.0f%% esecuri\n"
          % (len(X), X.shape[1], len(set(g)), 100 * y.mean()))

    candidates = [
        ("Baseline (clasa majoritara)", None),
        ("Regresie logistica",
         lambda: make_pipeline(StandardScaler(), LogisticRegression(max_iter=2000))),
        ("Retea neurala (64,32)",
         lambda: make_pipeline(StandardScaler(),
                               MLPClassifier((64, 32), max_iter=3000, random_state=0))),
        ("Retea neurala (100,100)",
         lambda: make_pipeline(StandardScaler(),
                               MLPClassifier((100, 100), max_iter=3000, random_state=0))),
        ("Random Forest", make_model),
        ("Gradient Boosting",
         lambda: GradientBoostingClassifier(n_estimators=300, max_depth=3,
                                            learning_rate=0.05, random_state=0)),
        ("HistGradientBoosting",
         lambda: HistGradientBoostingClassifier(max_iter=300, random_state=0)),
    ]

    print("%-30s %-8s %-8s %-9s %-7s %s"
          % ("model", "acurat.", "recall", "precizie", "AUC", "recall PM2+PM3"))
    for name, make in candidates:
        if make is None:
            pred = baseline_predict(y, meta, meta)
            auc = "  -  "
        else:
            splitter = GroupKFold(n_splits=5)
            prob = np.zeros(len(y))
            for train, test in splitter.split(X, y, g):
                model = make()
                model.fit(X[train], y[train])
                prob[test] = model.predict_proba(X[test])[:, 1]
            pred = (prob >= 0.5).astype(int)
            auc = "%.3f" % roc_auc_score(y, prob)
        print("%-30s %-8.1f %-8.1f %-9.1f %-7s %.1f"
              % (name, 100 * accuracy_score(y, pred),
                 100 * recall_score(y, pred, zero_division=0),
                 100 * precision_score(y, pred, zero_division=0), auc,
                 100 * recall_score(y[hard], pred[hard], zero_division=0)))


def cmd_final(root):
    from sklearn.metrics import (accuracy_score, precision_score, recall_score,
                                 roc_auc_score, confusion_matrix)

    Xtr, ytr, gtr, mtr = build(root + "batch_results_training.csv",
                               root + "features_training.txt")
    Xte, yte, gte, mte = build(root + "batch_results_validation.csv",
                               root + "features_validation.txt")

    print("antrenare: %d randuri, %d drumuri, %.0f%% esecuri"
          % (len(Xtr), len(set(gtr)), 100 * ytr.mean()))
    print("test     : %d randuri, %d drumuri, %.0f%% esecuri"
          % (len(Xte), len(set(gte)), 100 * yte.mean()))
    print("suprapunere drumuri: %d\n" % len(set(gtr) & set(gte)))

    model = make_model()
    model.fit(Xtr, ytr)
    prob = model.predict_proba(Xte)[:, 1]
    pred = (prob >= 0.5).astype(int)

    print("=== performanta pe drumuri complet noi ===")
    print("  acuratete=%.1f%%  precizie=%.1f%%  recall=%.1f%%  AUC=%.3f"
          % (100 * accuracy_score(yte, pred),
             100 * precision_score(yte, pred, zero_division=0),
             100 * recall_score(yte, pred, zero_division=0),
             roc_auc_score(yte, prob)))
    tn, fp, fn, tp = confusion_matrix(yte, pred).ravel()
    print("  succes corect=%d  fals alarma=%d  esec ratat=%d  esec prins=%d\n"
          % (tn, fp, fn, tp))
    per_model(yte, pred, mte)

    print("\n=== ECONOMIA DE SIMULARI ===")
    savings_table(prob, yte)

    base = baseline_predict(ytr, mtr, mte)
    print("\n  baseline fara geometrie: acuratete=%.1f%%  recall=%.1f%%"
          % (100 * accuracy_score(yte, base),
             100 * recall_score(yte, base, zero_division=0)))

    order = np.argsort(model.feature_importances_)[::-1][:8]
    print("\n  cele mai importante trasaturi:")
    for i in order:
        print("    %-18s %.3f" % (FEATURE_NAMES[i], model.feature_importances_[i]))


def cmd_train(root):
    X, y, g, _ = build(root + "batch_results_training.csv",
                       root + "features_training.txt")
    model = make_model()
    model.fit(X, y)
    with open(root + "surrogate.pkl", "wb") as handle:
        pickle.dump({"model": model, "features": FEATURE_NAMES}, handle)
    print("model antrenat pe %d randuri, salvat in surrogate.pkl" % len(X))


def cmd_regress(root):
    from sklearn.ensemble import GradientBoostingRegressor
    from sklearn.compose import TransformedTargetRegressor
    from sklearn.model_selection import GroupKFold
    from sklearn.metrics import r2_score
    from scipy.stats import spearmanr

    FITNESS = "f1_margin_to_oob"

    results = root + "batch_results_training.csv"
    X, _, g, meta = build(results, root + "features_training.txt")

    # re-read CSV with the same filters as build() so rows align
    roads = load_features(root + "features_training.txt")
    rows = [r for r in csv.DictReader(open(results))
            if r.get("width") and r["road"] in roads]
    if FITNESS not in rows[0]:
        print("Coloana '%s' nu exista. Coloane disponibile:" % FITNESS)
        print("  " + ", ".join(rows[0].keys()))
        print("Modifica FITNESS in cmd_regress cu numele corect.")
        return

    yr = np.array([float(r[FITNESS]) if r[FITNESS] not in ("", None) else np.nan
                   for r in rows])
    ok = ~np.isnan(yr)
    X, g, yr = X[ok], g[ok], yr[ok]
    models = np.array([m[2] for m in meta])[ok]
    print("set: %d randuri cu fitness valid, %d drumuri\n" % (len(yr), len(set(g))))

    def mlp(layers):
        # scale both inputs and target: MLP is sensitive to both
        return lambda: TransformedTargetRegressor(
            regressor=make_pipeline(StandardScaler(),
                                    MLPRegressor(hidden_layer_sizes=layers, max_iter=3000,
                                                 early_stopping=True,
                                                 random_state=0)),
            transformer=StandardScaler())

    candidates = [
        ("Gradient Boosting",
         lambda: GradientBoostingRegressor(n_estimators=300, max_depth=3,
                                           learning_rate=0.05, random_state=0)),
        ("Retea neurala (64,32)", mlp((64, 32))),
        ("Retea neurala (100,100)", mlp((100, 100))),
    ]

    print("%-26s %-10s %-10s %s" % ("model", "R2", "rho", "rho per model ENPS"))
    for name, make in candidates:
        pred = np.zeros(len(yr))
        for m in MODELS:
            idx = np.where(models == m)[0]
            if len(idx) == 0:
                continue
            for tr, te in GroupKFold(n_splits=5).split(X[idx], yr[idx], g[idx]):
                reg = make()
                reg.fit(X[idx[tr]], yr[idx[tr]])
                pred[idx[te]] = reg.predict(X[idx[te]])
        per = "  ".join("%s=%+.3f" % (m.replace("Model ", "M"),
                        spearmanr(pred[models == m], yr[models == m]).correlation)
                        for m in MODELS if (models == m).any())
        print("%-26s %-10.3f %-+10.3f %s"
              % (name, r2_score(yr, pred), spearmanr(pred, yr).correlation, per))


if __name__ == "__main__":
    command = sys.argv[1] if len(sys.argv) > 1 else "final"
    root = sys.argv[2] if len(sys.argv) > 2 else ""
    {"compare": cmd_compare, "final": cmd_final, "train": cmd_train,
     "regress": cmd_regress}[command](root)