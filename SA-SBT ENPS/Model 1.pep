num_ps = {
    # membrane names (labels)
    H = {
		SpeedLeft, SpeedRight,
		Compute1, Compute2, Compute3, Compute4, Compute5, Compute6,
		Sensor1, Sensor2, Sensor3, Sensor4, Sensor5, Sensor6,
		Weight1, Weight2, Weight3, Weight4, Weight5, Weight6,
		CruiseSpeed
	};

    structure = [SpeedLeft [SpeedRight [Compute1 ]Compute1  [Sensor1 ]Sensor1 [Weight1 ]Weight1
									   [Compute2 ]Compute2  [Sensor2 ]Sensor2 [Weight2 ]Weight2
									   [Compute3 ]Compute3  [Sensor3 ]Sensor3 [Weight3 ]Weight3
									   [Compute4 ]Compute4 [Sensor4 ]Sensor4 [Weight4 ]Weight4
									   [Compute5 ]Compute5  [Sensor5 ]Sensor5 [Weight5 ]Weight5
									   [Compute6 ]Compute6  [Sensor6 ]Sensor6 [Weight6 ]Weight6
									   [CruiseSpeed ]CruiseSpeed
							]SpeedRight 
				]SpeedLeft;

    # membrane 
    SpeedLeft = {
        var = {lw};
        pr = {0 * lw -> 1|lw};
        var0 = (0);
      };

    SpeedRight = {
        var = {rw};
        pr = {0 * rw -> 1|rw};
        var0 = (0);        
    };
    Compute1 = {
        var = {slval1, wl1, srval1, wr1};
		E = {ec1};
        pr = {slval1 * wl1 [ec1 -> ] 1|lw};
		pr = {srval1 * wr1 [ec1 -> ] 1|rw};
        var0 = (0, 0);
		E0 = (0);
    };
	
	Compute2 = {
        var = {slval2, wl2, srval2, wr2};
		E = {ec2};
        pr = {slval2 * wl2 [ec2 -> ] 1|lw};
		pr = {srval2 * wr2 [ec2 -> ] 1|rw};
        var0 = (0, 0);
		E0 = (0);
    };

   Compute3 = {
        var = {slval3, wl3, srval3, wr3};
		E = {ec3};
        pr = {slval3 * wl3 [ec3 -> ] 1|lw};
		pr = {srval3 * wr3 [ec3 -> ] 1|rw};
        var0 = (0, 0);
		E0 = (0);
    };

    Compute4 = {
        var = {slval4, wl4, srval4, wr4};
		E = {ec4};
        pr = {slval4 * wl4 [ec4 -> ] 1|lw};
		pr = {srval4 * wr4 [ec4 -> ] 1|rw};
        var0 = (0, 0);
		E0 = (0);
    };

   Compute5 = {
        var = {slval5, wl5, srval5, wr5};
		E = {ec5};
        pr = {slval5 * wl5 [ec5 -> ] 1|lw};
		pr = {srval5 * wr5 [ec5 -> ] 1|rw};
        var0 = (0, 0);
		E0 = (0);
    };

    Compute6 = {
        var = {slval6, wl6, srval6, wr6};
		E = {ec6};
        pr = {slval6 * wl6 [ec6 -> ] 1|lw};
		pr = {srval6 * wr6 [ec6 -> ] 1|rw};
        var0 = (0, 0);
		E0 = (0);
    };

  

	#sensor_membrane
	
	Sensor1 = {
        var = {s1};
        pr = {3 * s1 -> 1|s1 + 1|slval1 + 1|srval1};
        var0 = (0);
    };
	
	Sensor2 = {
        var = {s2};
        pr = {3 * s2 -> 1|s2 + 1|slval2 + 1|srval2};
        var0 = (0);
    };
		
	Sensor3 = {
        var = {s3};
        pr = {3 * s3 -> 1|s3 + 1|slval3 + 1|srval3};
        var0 = (0);
    };
	
	Sensor4 = {
        var = {s4};
        pr = {3 * s4 -> 1|s4 + 1|slval4 + 1|srval4};
        var0 = (0);
    };
	
	Sensor5 = {
        var = {s5};
        pr = {3 * s5 -> 1|s5 + 1|slval5 + 1|srval5};
        var0 = (0);
    };
	
	Sensor6 = {
        var = {s6};
        pr = {3 * s6 -> 1|s6 + 1|slval6 + 1|srval6};
        var0 = (0);
    };

	Weight1 = {
		E = {ew1};
        var = {weightLeft1, weightRight1};
        pr = {2 * weightLeft1 [ew1 -> ] 1|weightLeft1 + 1|wl1};
        pr = {2 * weightRight1 [ew1 -> ] 1|weightRight1 + 1|wr1};
        var0 = (0);
		E0 = (0);
    };

	Weight2 = {
		E = {ew2};
        var = {weightLeft2, weightRight2};
        pr = {2 * weightLeft2 [ew2 -> ] 1|weightLeft2 + 1|wl2};
        pr = {2 * weightRight2 [ew2 -> ] 1|weightRight2 + 1|wr2};
        var0 = (0);
		E0 = (0);
    };

	Weight3 = {
		E = {ew3};
        var = {weightLeft3, weightRight3};
        pr = {2 * weightLeft3 [ew3 -> ] 1|weightLeft3 + 1|wl3};
        pr = {2 * weightRight3 [ew3 -> ] 1|weightRight3 + 1|wr3};
        var0 = (0);
		E0 = (0);
    };

	Weight4 = {
		E = {ew4};
        var = {weightLeft4, weightRight4};
        pr = {2 * weightLeft4 [ew4 -> ] 1|weightLeft4 + 1|wl4};
        pr = {2 * weightRight4 [ew4 -> ] 1|weightRight4 + 1|wr4};
        var0 = (0);
		E0 = (0);
    };
	
	Weight5 = {
		E = {ew5};
        var = {weightLeft5, weightRight5};
        pr = {2 * weightLeft5 [ew5 -> ] 1|weightLeft5 + 1|wl5};
        pr = {2 * weightRight5 [ew5 -> ] 1|weightRight5 + 1|wr5};
        var0 = (0);
		E0 = (0);
    };

	Weight6 = {
		E = {ew6};
        var = {weightLeft6, weightRight6};
        pr = {2 * weightLeft6 [ew6 -> ] 1|weightLeft6 + 1|wl6};
        pr = {2 * weightRight6 [ew6 -> ] 1|weightRight6 + 1|wr6};
        var0 = (0);
		E0 = (0);
    };

	CruiseSpeed = {
        var = {cruiseSpeed};
        pr = {3 * cruiseSpeed -> 1|cruiseSpeed + 1|lw + 1|rw};
        var0 = (0);
    };
	
}

