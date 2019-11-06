#ifndef CONFIG_H_
#define CONFIG_H_

#define LIGHT_SLEEP			0
#define SENSORS_ACTIVE		1
#define ONLY_BT				2

#define BAUDRATE			115200

// FLASH_____________________
#define FLASH_SIZE				3500000 // 3.5 MB
#define VALUES_SET_ADDR			0
#define CO2_THRESH_ADDR			1
#define UVI_DUR_THRESH_ADDR		2
#define UVI_THRESH_ADDR			3
#define VOC_THRESH_ADDR			4
#define TEMP_THRESH_ADDR		5
#define STEP_GOAL_ADDR_LO		6
#define STEP_GOAL_ADDR_HI		7

// Store current FLASH storage index before going to sleep and every 150 min

// Steps are directly saved into the memory
#define STEPS_FLASH_ADDR_LO			8
#define STEPS_FLASH_ADDR_HI			9

// Index is absolute to IDX_START
#define CO2_FLASH_IDX_ADDR_LO		10
#define CO2_FLASH_IDX_ADDR_HI		11
#define VOC_FLASH_IDX_ADDR_LO		12
#define VOC_FLASH_IDX_ADDR_HI		13
#define UVI_FLASH_IDX_ADDR_LO		14
#define UVI_FLASH_IDX_ADDR_HI		15
#define TEMP_FLASH_IDX_ADDR_LO		16
#define TEMP_FLASH_IDX_ADDR_HI		17

// Measurement Frequency
#define UV_FREQ_ADDR_LO				18
#define UV_FREQ_ADDR_HI				19
#define AQ_FREQ_ADDR_LO				20
#define AQ_FREQ_ADDR_HI				21

// Storage size in RAM
#define CO2_ARRAY_SIZE		3
#define VOC_ARRAY_SIZE		3
#define TEMP_ARRAY_SIZE		3
#define UVI_ARRAY_SIZE		3

// Storage size in Flash in Bytes
#define CO2_STORAGE_SIZE		2000	// Possible to store measurements every 5 sec for 24 h ( = 17280). CO2 needs 2 Byte
#define VOC_STORAGE_SIZE		1000
#define TEMP_STORAGE_SIZE		1000
#define UVI_STORAGE_SIZE		1000

// Some buffer is left between the designated memories in case saving goes wrong.
#define CO2_FLASH_IDX_START			30
#define CO2_FLASH_IDX_STOP			2029
#define VOC_FLASH_IDX_START			2030
#define VOC_FLASH_IDX_STOP			3029
#define UVI_FLASH_IDX_START			3030
#define UVI_FLASH_IDX_STOP			4029

#define TEMP_FLASH_IDX_START		80000
#define TEMP_FLASH_IDX_STOP			99999


// _____________________

// Threshold durations for different skin types (min)
#define SKIN_TYPE_1				10
#define SKIN_TYPE_2				20
#define SKIN_TYPE_3				30
#define SKIN_TYPE_4				50
#define SKIN_TYPE_5				60	// actually >60


// Pins as in WCS_Version10
#define	VIBRATION_PIN			35

// LED pins
#define LEDBLUE_PIN				23 // Bluetooth
#define	LEDGREEN_PIN			16 // Power
#define LEDRED_PIN				15// Warning

// Button pins
#define POWER_PIN				36
#define BLUETOOTH_PIN			34

#define PRESSED_BUTTON_LEVEL	0

#define SENSORS_EN_PIN			13

#define SDA_PIN					22
#define SCL_PIN					21


// Frequencies
#define PEDO_FREQ				50	// in ms.
#define AQ_FREQ					3 	// in sec.
#define UV_FREQ					3	// in sec.
#define SHOW_FREQ				3	// in sec.

// Pedometer

#define Z_PIN 							32
#define PEDO_CALIBRATION_SAMPLE_SIZE 	15
#define PEDO_SAMPLE_SIZE 				15
#define ACCL							60000
#define WAIT_AFTER_STEP					500







#endif /* CONFIG_H_ */
