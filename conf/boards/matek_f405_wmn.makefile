# Hey Emacs, this is a -*- makefile -*-
#
# matek_f405_wmn.makefile
#
# http://www.mateksys.com/?portfolio=f405-wmn
#

BOARD=matek_f405_wmn
BOARD_CFG=\"boards/mateksys/F405-TE/$(BOARD).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/openpilot_revo.ld

# -----------------------------------------------------------------------

# default flash mode is via SWD
# other possibilities: DFU, DFU-UTIL, SWD, STLINK
FLASH_MODE ?= DFU-UTIL

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default uart configuration
#
SBUS_PORT ?= UART2

MODEM_PORT ?= UART5
MODEM_BAUD ?= B57600

GPS_PORT ?= UART4
GPS_BAUD ?= B57600

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
