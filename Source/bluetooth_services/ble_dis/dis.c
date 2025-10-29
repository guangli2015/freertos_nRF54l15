/*
 * Copyright (c) 2012 - 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <ble_gatts.h>
#include "uuid.h"
#include <common.h>
#include <stdbool.h>
#include "prj_config.h"
#define LOG_DBG
#define LOG_ERR
#define LOG_WRN
#define __ASSERT
#define ZERO_OR_COMPILE_ERROR(cond) ((int) sizeof(char[1 - 2 * !(cond)]) - 1)
/**
 * @brief Zero if @p array has an array type, a compile error otherwise
 *
 * This macro is available only from C, not C++.
 */
#define IS_ARRAY(array) \
	ZERO_OR_COMPILE_ERROR( \
		!__builtin_types_compatible_p(__typeof__(array), \
					      __typeof__(&(array)[0])))
#define ARRAY_SIZE(array) \
	((size_t) (IS_ARRAY(array) + (sizeof(array) / sizeof((array)[0]))))

#define SYS_ID_LEN 8 /* Length of System ID Characteristic Value */
#define PNP_ID_LEN 7 /* Length of PnP ID Characteristic Value */
#define IEEE_CERT_LEN 8 /* Maximum number of IEEE certifications */

#define gatt_char(_uuid, _value, ...)                                                              \
	{                                                                                          \
		.uuid = _uuid,                                                                     \
		.value = _value,                                                                   \
		COND_CODE_1(IS_EMPTY(__VA_ARGS__), (.len = strlen(_value),),                       \
			    (.len = __VA_ARGS__,))                                                 \
	}

#if CONFIG_BLE_DIS_SYSTEM_ID
static const uint8_t sys_id[SYS_ID_LEN] = {
	[0] = (CONFIG_BLE_DIS_SYSTEM_ID_MID & 0x00000000ff),
	[1] = (CONFIG_BLE_DIS_SYSTEM_ID_MID & 0x000000ff00) >> 8,
	[2] = (CONFIG_BLE_DIS_SYSTEM_ID_MID & 0x0000ff0000) >> 16,
	[3] = (CONFIG_BLE_DIS_SYSTEM_ID_MID & 0x00ff000000) >> 24,
	[4] = (CONFIG_BLE_DIS_SYSTEM_ID_MID & 0xff00000000) >> 32,
	[5] = (CONFIG_BLE_DIS_SYSTEM_ID_OUI & 0x0000ff),
	[6] = (CONFIG_BLE_DIS_SYSTEM_ID_OUI & 0x00ff00) >> 8,
	[7] = (CONFIG_BLE_DIS_SYSTEM_ID_OUI & 0xff0000) >> 16,
};
#endif

#if CONFIG_BLE_DIS_PNP_ID
static const uint8_t pnp_id[PNP_ID_LEN] = {
	[0] = (CONFIG_BLE_DIS_PNP_VID_SRC),
	[1] = (CONFIG_BLE_DIS_PNP_VID & 0x00ff),
	[2] = (CONFIG_BLE_DIS_PNP_VID & 0xff00) >> 8,
	[3] = (CONFIG_BLE_DIS_PNP_PID & 0x00ff),
	[4] = (CONFIG_BLE_DIS_PNP_PID & 0xff00) >> 8,
	[5] = (CONFIG_BLE_DIS_PNP_VER & 0x00ff),
	[6] = (CONFIG_BLE_DIS_PNP_VER & 0xff00) >> 8,
};
#endif

#if CONFIG_BLE_DIS_REGULATORY_CERT
static const uint8_t regulatory_certifications[IEEE_CERT_LEN] =
	sys_uint64_to_array(CONFIG_BLE_DIS_REGULATORY_CERT_LIST);
#endif



int ble_dis_init(void)
{
	int err;
	uint16_t service_handle;
	ble_uuid_t ble_uuid;
	ble_gatts_char_handles_t char_handles = {0};
	ble_gatts_char_md_t char_md = {
		.char_props = {
			.read = true,
		},
	};
	ble_gatts_attr_md_t attr_md = {
		.vloc = BLE_GATTS_VLOC_STACK,
		.read_perm = gap_conn_sec_mode_from_u8(CONFIG_BLE_DIS_CHAR_SEC_MODE),
	};
	ble_gatts_attr_t attr_char_value = {
		.p_uuid = &ble_uuid,
		.p_attr_md = &attr_md,
	};

	const struct {
		const uint8_t *value;
		uint16_t uuid;
		uint8_t len;
	} chars[] = {
		gatt_char(BLE_UUID_MANUFACTURER_NAME_STRING_CHAR, CONFIG_BLE_DIS_MANUFACTURER_NAME),
		gatt_char(BLE_UUID_MODEL_NUMBER_STRING_CHAR, CONFIG_BLE_DIS_MODEL_NUMBER),
		gatt_char(BLE_UUID_SERIAL_NUMBER_STRING_CHAR, CONFIG_BLE_DIS_SERIAL_NUMBER),
		gatt_char(BLE_UUID_HARDWARE_REVISION_STRING_CHAR, CONFIG_BLE_DIS_HW_REVISION),
		gatt_char(BLE_UUID_FIRMWARE_REVISION_STRING_CHAR, CONFIG_BLE_DIS_FW_REVISION),
		gatt_char(BLE_UUID_SOFTWARE_REVISION_STRING_CHAR, CONFIG_BLE_DIS_SW_REVISION),
#if CONFIG_BLE_DIS_SYSTEM_ID
		gatt_char(BLE_UUID_SYSTEM_ID_CHAR, sys_id, sizeof(sys_id)),
#endif
#if CONFIG_BLE_DIS_PNP_ID
		gatt_char(BLE_UUID_PNP_ID_CHAR, pnp_id, sizeof(pnp_id)),
#endif
#if CONFIG_BLE_DIS_REGULATORY_CERT
		gatt_char(BLE_UUID_IEEE_REGULATORY_CERTIFICATION_DATA_LIST_CHAR,
			  regulatory_certifications, sizeof(regulatory_certifications)),
#endif
	};

	/* Add service */
	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_DEVICE_INFORMATION_SERVICE);

	err = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
	if (err) {
		LOG_ERR("Failed to add device information service, nrf_error %#x", err);
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(chars); i++) {
		if (!chars[i].len) {
			/* If the user didn't add any */
			continue;
		}

		BLE_UUID_BLE_ASSIGN(ble_uuid, chars[i].uuid);
		attr_char_value.p_value = (uint8_t *)chars[i].value;
		attr_char_value.max_len = chars[i].len;
		attr_char_value.init_len = chars[i].len;

		LOG_DBG("Added char %#x, len %d", chars[i].uuid, chars[i].len);

		err = sd_ble_gatts_characteristic_add(service_handle, &char_md,
						      &attr_char_value, &char_handles);
		if (err) {
			LOG_ERR("Failed to add characteristic, nrf_error %#x", err);
			return -EINVAL;
		}
	}

	return 0;
}
