# Soil-Irrigation-System
Creating an automatic plant watering system using STM32 Microcontrollers to detect soil moisture. 

The irrigation system will monitor soil moisture levels using two STM-32 microcontrollers and sensors to control the water pump to ensure optimal watering based on soil moisture content and environmental factors. The system will have sensors, controllers, and status indicators to automate irrigation. 

Microcontroller #1 will continuously monitor the soil moisture levels via the soil moisture sensor. Once the moisture levels falls below its predefined threshold, the microcontroller will send a signal to controller #2 to turn off a white LED continuously for a maximum limit of 24 hours. This white LED signals that the plant needs watering.

If the plant is watered within the 24-hour period and the soil moisture level rises to the acceptable threshold, Microcontroller #1 will send a signal to Microcontroller #2 to:

Turn off the white LED
Turn on the green LED, indicating that the plant has been sufficiently watered

However, if the soil moisture level does not reach the required threshold within 24 hours, Microcontroller #1 will signal Microcontroller #2 to:

Turn off the white LED
Turn on the red LED to indicate that the water pump has been activated

Once the correct soil moisture level is reached, Microcontroller #1 will signal Microcontroller #2 to:
Turn off the red LED
Turn on the green LED, indicating that the irrigation is complete and the soil has the correct moisture level.
