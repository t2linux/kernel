// SPDX-License-Identifier: ISC
/*
 * Copyright The Asahi Linux Contributors
 */

#include <linux/acpi.h>
#include "debug.h"
#include "core.h"
#include "common.h"

void brcmf_acpi_probe(struct device *dev, enum brcmf_bus_type bus_type,
		    struct brcmf_mp_device *settings)
{
	acpi_status status;
	struct acpi_device *adev = ACPI_COMPANION(dev);
	const union acpi_object *o;
	struct acpi_buffer buf = {ACPI_ALLOCATE_BUFFER, NULL};

	if (!adev) {
		return;
	}

	if (!ACPI_FAILURE(acpi_dev_get_property(adev, "module-instance",
		ACPI_TYPE_STRING, &o))) {
		const char *prefix = "apple,";
		int len = strlen(prefix) + o->string.length + 1;
		char *board_type = devm_kzalloc(dev, len, GFP_KERNEL);

		strlcpy(board_type, prefix, len);
		strlcat(board_type, o->string.pointer, len);
		brcmf_dbg(INFO, "ACPI module-instance=%s\n", o->string.pointer);
		settings->board_type = board_type;
	} else {
		brcmf_dbg(INFO, "No ACPI module-instance\n");
	}

	status = acpi_evaluate_object(adev->handle, "RWCV", NULL, &buf);
	o = buf.pointer;
	if (!ACPI_FAILURE(status) && o && o->type == ACPI_TYPE_BUFFER &&
		o->buffer.length >= 2) {
		char *antenna_sku = devm_kzalloc(dev, 3, GFP_KERNEL);

		memcpy(antenna_sku, o->buffer.pointer, 2);
		brcmf_dbg(INFO, "ACPI RWCV data=%*phN antenna-sku=%s\n",
			  (int)o->buffer.length, o->buffer.pointer,
			  antenna_sku);

		settings->antenna_sku = antenna_sku;
	} else {
		brcmf_dbg(INFO, "No ACPI antenna-sku\n");
	}
}
