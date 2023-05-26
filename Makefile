# https://stackoverflow.com/questions/1079832/how-can-i-configure-my-makefile-for-debug-and-release-builds

# SHELL := cmd.exe
RM := rm -rf

PROJECT = bt_soc_blinky
BUILD_DIR = build
OBJ_SUBDIR = obj
OUTPUT_FILE_PATH := $(BUILD_DIR)/$(PROJECT).elf

ifeq ($(OS),Windows_NT)
#	WINDOWS_TOOLCHAIN_PATH = "C:/Program Files (x86)/Atmel/Studio/7.0/toolchain/arm/arm-gnu-toolchain/bin"
	WINDOWS_TOOLCHAIN_PATH = "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin"
	C_COMPILER = $(WINDOWS_TOOLCHAIN_PATH)/arm-none-eabi-gcc.exe
	CPP_COMPILER = $(WINDOWS_TOOLCHAIN_PATH)/arm-none-eabi-g++.exe
	ASM_COMPILER = $(WINDOWS_TOOLCHAIN_PATH)/arm-none-eabi-as
	OBJCOPY = $(WINDOWS_TOOLCHAIN_PATH)/arm-none-eabi-objcopy
	OBJSIZE = $(WINDOWS_TOOLCHAIN_PATH)/arm-none-eabi-size
else
	C_COMPILER = arm-none-eabi-gcc
	CPP_COMPILER = arm-none-eabi-g++
	ASM_COMPILER = arm-none-eabi-as
	OBJCOPY = arm-none-eabi-objcopy
	OBJSIZE = arm-none-eabi-size
endif

SOURCE_DIR = .
COMMON_LIB_DIR = ../app_libs
GECKO_SDK = /Users/denissuprunenko/SimplicityStudio/SDKs/gecko_sdk
# GECKO_SDK = C:/Users/dSuprunenko/repos/gecko_sdk

CPU_OPTIONS = -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
C_STANDARD = -std=c99
CPP_STANDARD = -std=c++17
COMMON_OPTIONS = -g3 -gdwarf-2 -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mcmse --specs=nano.specs -c -fmessage-length=0

DEFINES = \
'-DBGM220PC22HNA=1' \
'-DSL_APP_PROPERTIES=1' \
'-DBOOTLOADER_APPLOADER=1' \
'-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' \
'-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' \
'-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' \
'-DSL_BOARD_NAME="BRD4314A"' \
'-DSL_BOARD_REV="A02"' \
'-DSL_COMPONENT_CATALOG_PRESENT=1' \
'-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' \
'-DSLI_RADIOAES_REQUIRES_MASKING=1' \
'-DMBEDTLS_CONFIG_FILE=<sl_mbedtls_config.h>' \
'-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' \
'-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' \

LD_SCRIPT = $(SOURCE_DIR)/autogen/linkerfile.ld

LIBS := \
$(GECKO_SDK)/protocol/bluetooth/lib/EFR32BG22/GCC/libbluetooth.a \
$(GECKO_SDK)/platform/emdrv/nvm3/lib/libnvm3_CM33_gcc.a \
$(GECKO_SDK)/platform/radio/rail_lib/autogen/librail_release/librail_config_bgm220pc22hna_gcc.a \
$(GECKO_SDK)/platform/radio/rail_lib/autogen/librail_release/librail_module_efr32xg22_gcc_release.a \
-lgcc -lc -lnosys \

ASM_OPTIONS =
C_OPTIONS   = $(COMMON_OPTIONS) $(C_STANDARD) $(CPU_OPTIONS)
CPP_OPTIONS = $(COMMON_OPTIONS) $(CPP_STANDARD) $(CPU_OPTIONS) -fno-exceptions -fno-rtti

ASM_INCLUDE =

C_INCLUDE = \
-I$(SOURCE_DIR)/config \
-I$(SOURCE_DIR)/config/btconf \
-I$(SOURCE_DIR) \
-I$(GECKO_SDK)/platform/Device/SiliconLabs/BGM22/Include \
-I$(GECKO_SDK)/app/common/util/app_assert \
-I$(GECKO_SDK)/app/common/util/app_log \
-I$(GECKO_SDK)/protocol/bluetooth/inc \
-I$(GECKO_SDK)/platform/common/inc \
-I$(GECKO_SDK)/hardware/board/inc \
-I$(GECKO_SDK)/platform/bootloader \
-I$(GECKO_SDK)/platform/bootloader/api \
-I$(GECKO_SDK)/platform/driver/button/inc \
-I$(GECKO_SDK)/platform/CMSIS/Core/Include \
-I$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/include \
-I$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src \
-I$(GECKO_SDK)/platform/service/device_init/inc \
-I$(GECKO_SDK)/platform/emdrv/dmadrv/inc \
-I$(GECKO_SDK)/platform/emdrv/common/inc \
-I$(GECKO_SDK)/platform/emlib/inc \
-I$(GECKO_SDK)/platform/radio/rail_lib/plugin/fem_util \
-I$(GECKO_SDK)/app/bluetooth/common/gatt_service_device_information \
-I$(GECKO_SDK)/platform/emdrv/gpiointerrupt/inc \
-I$(GECKO_SDK)/platform/service/hfxo_manager/inc \
-I$(GECKO_SDK)/app/bluetooth/common/in_place_ota_dfu \
-I$(GECKO_SDK)/platform/service/iostream/inc \
-I$(GECKO_SDK)/platform/driver/leddrv/inc \
-I$(GECKO_SDK)/platform/security/sl_component/sl_mbedtls_support/config \
-I$(GECKO_SDK)/platform/security/sl_component/sl_mbedtls_support/inc \
-I$(GECKO_SDK)/util/third_party/mbedtls/include \
-I$(GECKO_SDK)/util/third_party/mbedtls/library \
-I$(GECKO_SDK)/platform/service/mpu/inc \
-I$(GECKO_SDK)/platform/emdrv/nvm3/inc \
-I$(GECKO_SDK)/platform/service/power_manager/inc \
-I$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/inc \
-I$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/inc/public \
-I$(GECKO_SDK)/platform/radio/rail_lib/common \
-I$(GECKO_SDK)/platform/radio/rail_lib/protocol/ble \
-I$(GECKO_SDK)/platform/radio/rail_lib/protocol/ieee802154 \
-I$(GECKO_SDK)/platform/radio/rail_lib/protocol/zwave \
-I$(GECKO_SDK)/platform/radio/rail_lib/chip/efr32/efr32xg2x \
-I$(GECKO_SDK)/platform/radio/rail_lib/plugin/pa-conversions \
-I$(GECKO_SDK)/platform/radio/rail_lib/plugin/pa-conversions/efr32xg22 \
-I$(GECKO_SDK)/platform/radio/rail_lib/plugin/rail_util_pti \
-I$(GECKO_SDK)/platform/security/sl_component/se_manager/inc \
-I$(GECKO_SDK)/platform/security/sl_component/se_manager/src \
-I$(GECKO_SDK)/util/silicon_labs/silabs_core/memory_manager \
-I$(GECKO_SDK)/platform/common/toolchain/inc \
-I$(GECKO_SDK)/platform/service/system/inc \
-I$(GECKO_SDK)/platform/service/sleeptimer/inc \
-I$(GECKO_SDK)/platform/security/sl_component/sl_protocol_crypto/src \
-I$(SOURCE_DIR)/autogen \

CPP_INCLUDE =

ASMFILES = 

CFILES = \
$(GECKO_SDK)/util/third_party/mbedtls/library/cipher_wrap.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/cipher.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/constant_time.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/platform.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/platform_util.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_aead.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_cipher.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_client.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_driver_wrappers.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_ecp.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_hash.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_mac.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_rsa.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_se.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_slot_management.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/psa_crypto_storage.c \
$(GECKO_SDK)/util/third_party/mbedtls/library/threading.c \
$(GECKO_SDK)/util/silicon_labs/silabs_core/memory_manager/sl_malloc.c \
$(GECKO_SDK)/protocol/bluetooth/src/sl_apploader_util_s2.c \
$(GECKO_SDK)/protocol/bluetooth/src/sl_bt_stack_init.c \
$(GECKO_SDK)/protocol/bluetooth/src/sli_bt_advertiser_config.c \
$(GECKO_SDK)/protocol/bluetooth/src/sli_bt_channel_sounding_config.c \
$(GECKO_SDK)/protocol/bluetooth/src/sli_bt_connection_config.c \
$(GECKO_SDK)/protocol/bluetooth/src/sli_bt_dynamic_gattdb_config.c \
$(GECKO_SDK)/protocol/bluetooth/src/sli_bt_l2cap_config.c \
$(GECKO_SDK)/protocol/bluetooth/src/sli_bt_pawr_advertiser_config.c \
$(GECKO_SDK)/protocol/bluetooth/src/sli_bt_periodic_adv_config.c \
$(GECKO_SDK)/protocol/bluetooth/src/sli_bt_periodic_advertiser_config.c \
$(GECKO_SDK)/protocol/bluetooth/src/sli_bt_sync_config.c \
$(GECKO_SDK)/platform/service/system/src/sl_system_process_action.c \
$(GECKO_SDK)/platform/service/system/src/sl_system_init.c \
$(GECKO_SDK)/platform/service/sleeptimer/src/sl_sleeptimer.c \
$(GECKO_SDK)/platform/service/sleeptimer/src/sl_sleeptimer_hal_burtc.c \
$(GECKO_SDK)/platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.c \
$(GECKO_SDK)/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.c \
$(GECKO_SDK)/platform/service/sleeptimer/src/sl_sleeptimer_hal_timer.c \
$(GECKO_SDK)/platform/service/power_manager/src/sl_power_manager.c \
$(GECKO_SDK)/platform/service/power_manager/src/sl_power_manager_debug.c \
$(GECKO_SDK)/platform/service/power_manager/src/sl_power_manager_hal_s2.c \
$(GECKO_SDK)/platform/service/mpu/src/sl_mpu.c \
$(GECKO_SDK)/platform/service/iostream/src/sl_iostream.c \
$(GECKO_SDK)/platform/service/iostream/src/sl_iostream_retarget_stdio.c \
$(GECKO_SDK)/platform/service/iostream/src/sl_iostream_stdlib_config.c \
$(GECKO_SDK)/platform/service/iostream/src/sl_iostream_uart.c \
$(GECKO_SDK)/platform/service/iostream/src/sl_iostream_usart.c \
$(GECKO_SDK)/platform/service/hfxo_manager/src/sl_hfxo_manager.c \
$(GECKO_SDK)/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.c \
$(GECKO_SDK)/platform/service/device_init/src/sl_device_init_dcdc_s2.c \
$(GECKO_SDK)/platform/service/device_init/src/sl_device_init_emu_s2.c \
$(GECKO_SDK)/platform/service/device_init/src/sl_device_init_hfxo_s2.c \
$(GECKO_SDK)/platform/service/device_init/src/sl_device_init_lfrco.c \
$(GECKO_SDK)/platform/service/device_init/src/sl_device_init_lfxo_s2.c \
$(GECKO_SDK)/platform/service/device_init/src/sl_device_init_nvic.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/cryptoacc_management.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sl_psa_its_nvm3.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_cryptoacc_transparent_driver_aead.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_cryptoacc_transparent_driver_cipher.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_cryptoacc_transparent_driver_hash.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_cryptoacc_transparent_driver_key_derivation.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_cryptoacc_transparent_driver_key_management.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_cryptoacc_transparent_driver_mac.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_cryptoacc_transparent_driver_signature.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_psa_driver_common.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_psa_driver_init.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_psa_trng.c \
$(GECKO_SDK)/platform/security/sl_component/sl_psa_driver/src/sli_se_version_dependencies.c \
$(GECKO_SDK)/platform/security/sl_component/sl_protocol_crypto/src/sli_protocol_crypto_radioaes.c \
$(GECKO_SDK)/platform/security/sl_component/sl_protocol_crypto/src/sli_radioaes_management.c \
$(GECKO_SDK)/platform/security/sl_component/sl_mbedtls_support/src/cryptoacc_aes.c \
$(GECKO_SDK)/platform/security/sl_component/sl_mbedtls_support/src/cryptoacc_gcm.c \
$(GECKO_SDK)/platform/security/sl_component/sl_mbedtls_support/src/mbedtls_ccm.c \
$(GECKO_SDK)/platform/security/sl_component/sl_mbedtls_support/src/mbedtls_cmac.c \
$(GECKO_SDK)/platform/security/sl_component/sl_mbedtls_support/src/mbedtls_ecdsa_ecdh.c \
$(GECKO_SDK)/platform/security/sl_component/sl_mbedtls_support/src/sl_mbedtls.c \
$(GECKO_SDK)/platform/security/sl_component/sl_mbedtls_support/src/sli_psa_crypto.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/ba414ep_config.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/ba431_config.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/cryptodma_internal.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/cryptolib_types.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_aes.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_blk_cipher.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_dh_alg.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_ecc_curves.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_ecc_keygen_alg.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_ecdsa_alg.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_hash.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_math.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_memcmp.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_memcpy.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_primitives.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_rng.c \
$(GECKO_SDK)/platform/security/sl_component/sl_cryptoacc_library/src/sx_trng.c \
$(GECKO_SDK)/platform/security/sl_component/se_manager/src/sl_se_manager_attestation.c \
$(GECKO_SDK)/platform/security/sl_component/se_manager/src/sl_se_manager.c \
$(GECKO_SDK)/platform/security/sl_component/se_manager/src/sl_se_manager_cipher.c \
$(GECKO_SDK)/platform/security/sl_component/se_manager/src/sl_se_manager_entropy.c \
$(GECKO_SDK)/platform/security/sl_component/se_manager/src/sl_se_manager_hash.c \
$(GECKO_SDK)/platform/security/sl_component/se_manager/src/sl_se_manager_key_derivation.c \
$(GECKO_SDK)/platform/security/sl_component/se_manager/src/sl_se_manager_key_handling.c \
$(GECKO_SDK)/platform/security/sl_component/se_manager/src/sl_se_manager_signature.c \
$(GECKO_SDK)/platform/security/sl_component/se_manager/src/sl_se_manager_util.c \
$(GECKO_SDK)/platform/radio/rail_lib/plugin/rail_util_pti/sl_rail_util_pti.c \
$(GECKO_SDK)/platform/radio/rail_lib/plugin/pa-conversions/pa_conversions_efr32.c \
$(GECKO_SDK)/platform/radio/rail_lib/plugin/fem_util/sl_fem_util.c \
$(GECKO_SDK)/platform/emlib/src/em_burtc.c \
$(GECKO_SDK)/platform/emlib/src/em_cmu.c \
$(GECKO_SDK)/platform/emlib/src/em_core.c \
$(GECKO_SDK)/platform/emlib/src/em_emu.c \
$(GECKO_SDK)/platform/emlib/src/em_gpio.c \
$(GECKO_SDK)/platform/emlib/src/em_ldma.c \
$(GECKO_SDK)/platform/emlib/src/em_msc.c \
$(GECKO_SDK)/platform/emlib/src/em_prs.c \
$(GECKO_SDK)/platform/emlib/src/em_rtcc.c \
$(GECKO_SDK)/platform/emlib/src/em_se.c \
$(GECKO_SDK)/platform/emlib/src/em_system.c \
$(GECKO_SDK)/platform/emlib/src/em_timer.c \
$(GECKO_SDK)/platform/emlib/src/em_usart.c \
$(GECKO_SDK)/platform/emdrv/nvm3/src/nvm3_default_common_linker.c \
$(GECKO_SDK)/platform/emdrv/nvm3/src/nvm3_hal_flash.c \
$(GECKO_SDK)/platform/emdrv/gpiointerrupt/src/gpiointerrupt.c \
$(GECKO_SDK)/platform/emdrv/nvm3/src/nvm3_lock.c \
$(GECKO_SDK)/platform/emdrv/dmadrv/src/dmadrv.c \
$(GECKO_SDK)/platform/driver/leddrv/src/sl_led.c \
$(GECKO_SDK)/platform/driver/leddrv/src/sl_simple_led.c \
$(GECKO_SDK)/platform/driver/button/src/sl_button.c \
$(GECKO_SDK)/platform/driver/button/src/sl_simple_button.c \
$(GECKO_SDK)/platform/common/toolchain/src/sl_memory.c \
$(GECKO_SDK)/platform/common/src/sl_assert.c \
$(GECKO_SDK)/platform/common/src/sl_string.c \
$(GECKO_SDK)/platform/common/src/sl_slist.c \
$(GECKO_SDK)/platform/bootloader/app_properties/app_properties.c \
$(GECKO_SDK)/platform/bootloader/api/btl_interface.c \
$(GECKO_SDK)/platform/bootloader/api/btl_interface_storage.c \
$(GECKO_SDK)/platform/Device/SiliconLabs/BGM22/Source/startup_bgm22.c \
$(GECKO_SDK)/platform/Device/SiliconLabs/BGM22/Source/system_bgm22.c \
$(GECKO_SDK)/hardware/board/src/sl_board_control_gpio.c \
$(GECKO_SDK)/hardware/board/src/sl_board_init.c \
$(GECKO_SDK)/app/common/util/app_log/app_log.c \
$(GECKO_SDK)/app/bluetooth/common/in_place_ota_dfu/sl_bt_in_place_ota_dfu.c \
$(SOURCE_DIR)/autogen/gatt_db.c \
$(SOURCE_DIR)/autogen/sl_bluetooth.c \
$(SOURCE_DIR)/autogen/sl_board_default_init.c \
$(SOURCE_DIR)/autogen/sl_device_init_clocks.c \
$(SOURCE_DIR)/autogen/sl_event_handler.c \
$(SOURCE_DIR)/autogen/sl_iostream_handles.c \
$(SOURCE_DIR)/autogen/sl_iostream_init_usart_instances.c \
$(SOURCE_DIR)/autogen/sl_power_manager_handler.c \
$(SOURCE_DIR)/autogen/sl_simple_button_instances.c \
$(SOURCE_DIR)/autogen/sl_simple_led_instances.c \
$(SOURCE_DIR)/app.c \
$(SOURCE_DIR)/main.c \
$(SOURCE_DIR)/sl_gatt_service_device_information.c \

CPPFILES =

SOURCE_DIRS := $(dir $(CFILES))
SOURCE_DIRS += $(dir $(CPPFILES))
SOURCE_DIRS += $(dir $(ASMFILES))

VPATH = $(sort $(SOURCE_DIRS))

C_FILENAMES := $(notdir $(CFILES))
CPP_FILENAMES := $(notdir $(CPPFILES))
ASM_FILENAMES := $(notdir $(ASMFILES))

OBJ_FILES := $(patsubst %.c, $(BUILD_DIR)/$(OBJ_SUBDIR)/%.o, $(C_FILENAMES) )
OBJ_FILES += $(patsubst %.cpp, $(BUILD_DIR)/$(OBJ_SUBDIR)/%.o, $(CPP_FILENAMES) )
OBJ_FILES += $(patsubst %.S, $(BUILD_DIR)/$(OBJ_SUBDIR)/%.o, $(ASM_FILENAMES) )

C_DEPS := $(OBJ_FILES:%.o=%.d)

.PHONY: all clean init

# All Target
all: init $(OUTPUT_FILE_PATH)

init:
	mkdir -p $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)/$(OBJ_SUBDIR)

$(BUILD_DIR)/$(OBJ_SUBDIR)/%.o : %.S
	$(info $<)
	@$(ASM_COMPILER) $(DEFINES) $(ASM_INCLUDE) $(ASM_OPTIONS) -MD -MP -MF $(@:%.o=%.d) -MT$(@:%.o=%.d) -MT$(@:%.o=%.o) -o $@ $< 

$(BUILD_DIR)/$(OBJ_SUBDIR)/%.o : %.c
	$(info $<)
	@$(C_COMPILER) $(DEFINES) $(C_INCLUDE) $(C_OPTIONS) -MD -MP -MF $(@:%.o=%.d) -MT$(@:%.o=%.d) -MT$(@:%.o=%.o) -o $@ $< 

$(BUILD_DIR)/$(OBJ_SUBDIR)/%.o : %.cpp
	$(info $<)
	@$(CPP_COMPILER) $(DEFINES) $(CPP_INCLUDE) $(CPP_OPTIONS) -MD -MP -MF $(@:%.o=%.d) -MT$(@:%.o=%.d) -MT$(@:%.o=%.o) -o $@ $< 

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

$(OUTPUT_FILE_PATH): $(OBJ_FILES)
	$(info Building target: $@)
	@$(CPP_COMPILER) -g3 -gdwarf-2 $(CPU_OPTIONS) -T $(LD_SCRIPT) -Xlinker --gc-sections -Xlinker -Map=$(BUILD_DIR)/$(PROJECT).map --specs=nano.specs -o $(OUTPUT_FILE_PATH) -Wl,--start-group $(OBJ_FILES) $(LIBS) -Wl,--end-group
	$(info Finished building target: $@)
	@$(OBJCOPY) -O binary $(OUTPUT_FILE_PATH) $(BUILD_DIR)/$(PROJECT).bin
	@$(OBJCOPY) -h -S $(OUTPUT_FILE_PATH) > $(BUILD_DIR)/$(PROJECT).lss
	@$(OBJSIZE) $(OUTPUT_FILE_PATH)

# Other Targets
clean:
	$(info $(C_FILENAMES))
	-$(RM) $(OBJ_FILES)
	-$(RM) $(C_DEPS)   
	-$(RM) $(OUTPUT_FILE_PATH) $(BUILD_DIR)/$(PROJECT).a $(BUILD_DIR)/$(PROJECT).hex $(BUILD_DIR)/$(PROJECT).bin $(BUILD_DIR)/$(PROJECT).lss $(BUILD_DIR)/$(PROJECT).eep $(BUILD_DIR)/$(PROJECT).map $(BUILD_DIR)/$(PROJECT).srec
