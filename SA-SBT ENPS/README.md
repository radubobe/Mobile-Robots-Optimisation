# Surrogate-Assisted Search-Based Testing of ENPS Robot Controllers

Predicting test outcomes of an E-puck obstacle-avoidance
controller based on Enzymatic Numerical P systems (ENPS), without running Webots.


## Repository structure

```
data/                          road spines exported from AmbieGen (one .txt per road)
enps_controller/   			   Webots controller; uses PeP to execute the ENPS model
model 1, model 2               ENPS model instances; 
batch_runner.py                runs (road × width × model) combinations in headless Webots
batch_traces/                  PeP execution traces and GPS paths archived per run
batch_results_training.csv     oracle verdicts + fitness values for the 100 training roads
batch_results_validation.csv   same for the 50 validation roads
sensor_model.py                geometric IR sensor reconstruction and differential-drive kinematics
surrogate.py                   black-box Random Forest classifier and regression experiments
TO BE UPDATED
```



## Dependencies

- Python 3.11 — `pip install scikit-learn numpy scipy`
- [Webots R2025a](https://cyberbotics.com) for the simulation campaign

The surrogate scripts (`surrogate.py`, `sensor_model.py`) have no Webots dependency
and run on any Python 3.8+ environment.

The Webots version used for generating traces was `R2025a` but the approach is also compatible with older Webots releases

