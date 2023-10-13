float32 battery_voltage  # Voltage in Volts 
float32 load_voltage  # Voltage in Volts 
float32 charger_voltage  # Voltage in Volts 
float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack

float32 temperature_bms      # Temperature in Degrees Celsius (If unmeasured NaN)
float32[] temperature_cell  # An array of individual cell temperatures for each cell in the pack

float32 current_income     # Pile gelen akım      
float32 capacity           # Pilin doluluk kapasitesi -> Capacity in Ah (last full capacity) 
float32 capacity_max  	    # Pilin total kapasitesi -> Capacity in Ah (design capacity)  
float32 soc                # Charge percentage on 0 to 1 range -> percentage
float32 current_charger    # A     
float32 current_load       # Sistemin Çektiği Akım -> Negative when discharging (A)  