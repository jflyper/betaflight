H743xI_TARGETS += $(TARGET)
#FEATURES       += SDCARD VCP
FEATURES       += VCP

# Top level Makefile adds, if not defined, HSE_VALUE, as default for F4 targets.
# We don't want to assume any particular value until de facto design is established,
# so we set the value here.
#
# However, HSE_VALUE is currently a global build option and can not be changed from
# board to board. Generic target will have to store this value as a PG variable and
# change clock on the fly after the PG became readable.

HSE_VALUE    = 8000000 # For NUCLEO-H743ZI with STLINK, HSE is 8MHz from STLINK

TARGET_SRC = \
#            drivers/accgyro/accgyro_fake.c \
#            drivers/accgyro/accgyro_mpu6050.c \
#            drivers/barometer/barometer_fake.c \
#            drivers/barometer/barometer_ms5611.c \
#            drivers/compass/compass_fake.c \
#            drivers/compass/compass_hmc5883l.c
