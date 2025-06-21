from zigpy.quirks import CustomCluster
from zigpy.quirks.v2 import QuirkBuilder, ReportingConfig
from zigpy.quirks.v2.homeassistant import EntityType
from zigpy.zcl.clusters.general import AnalogInput, MultistateValue, BinaryInput
import zigpy.types as t
from zigpy.quirks.v2.homeassistant.sensor import SensorDeviceClass, SensorStateClass

from zha.application.platforms.number.const import NumberMode
from zigpy.quirks.v2.homeassistant.number import NumberDeviceClass

import functools
from zha.application import Platform
from zha.application.registries import PLATFORM_ENTITIES
from zha.application.platforms.sensor import Sensor
from zha.application.platforms.select import ZCLEnumSelectEntity
from zha.zigbee.cluster_handlers.const import CLUSTER_HANDLER_ANALOG_INPUT
from zha.zigbee.cluster_handlers import ClusterHandler
MULTI_MATCH = functools.partial(PLATFORM_ENTITIES.multipass_match, Platform.SENSOR)

from zigpy.quirks.v2 import CustomDeviceV2
from zha.zigbee.cluster_handlers.const import REPORT_CONFIG_ASAP
from zha.zigbee.cluster_handlers import (
    AttrReportConfig,
    registries,
)
from zha.zigbee.cluster_handlers.general import AnalogInputClusterHandler, BinaryInputClusterHandler


DS2_QUIRK_ID = "ds_model2_quirk"

#
# Increase reporting rate for the input clusters
#
@registries.CLUSTER_HANDLER_REGISTRY.register(
    BinaryInput.cluster_id, DS2_QUIRK_ID
)
class dsBinaryInputClusterHandler(BinaryInputClusterHandler):
    REPORT_CONFIG = (
        AttrReportConfig(attr="present_value", config=REPORT_CONFIG_ASAP),
    )

@registries.CLUSTER_HANDLER_REGISTRY.register(
    AnalogInput.cluster_id, DS2_QUIRK_ID
)
class dsAnalogInputClusterHandler(AnalogInputClusterHandler):
    REPORT_CONFIG = (
        AttrReportConfig(attr="present_value", config=REPORT_CONFIG_ASAP),
    )


#
#Make ZHA recognize the sensor
#

@MULTI_MATCH(
    cluster_handler_names=CLUSTER_HANDLER_ANALOG_INPUT,
    manufacturers="DS",
    quirk_ids = DS2_QUIRK_ID,
    stop_on_match_group=CLUSTER_HANDLER_ANALOG_INPUT,
)
class myAnalogInput(Sensor):

    _attribute_name = "present_value"
    _attr_translation_key: str = "analog_input"


class lightOperationModes(t.enum8):
    """Light operation modes."""

    Auto = 0x01
    On = 0x02
    Off = 0x03


# ZHA sets an incorrect data type for "present_value" attribute,
# single precision float instead of 16-bit unsigned integer.

class m2MultistateValue(CustomCluster, MultistateValue):
    ep_attribute = "m2_multistate_value"
    attributes = MultistateValue.attributes.copy()
    attributes.update(
        {
            0x0055: ("present_value", t.uint16_t)
        }
    )

#
# Make ZHA recognize the tri-state switch
#

@MULTI_MATCH(
    cluster_handler_names="m2_multistate_value",
    manufacturers="DS",
    stop_on_match_group="m2_multistate_value",
)
class myMultistateValue(ZCLEnumSelectEntity):

    _attribute_name = "present_value"
    _attr_translation_key: str = "multistate_value"
    _enum = lightOperationModes
    _attr_entity_category = None

CALIB_TIME=30

#
# Apply the quirk
#
class ds2QuirkDevice(CustomDeviceV2):
    quirk_id = DS2_QUIRK_ID

(
    QuirkBuilder("DS", "Model2")
    .device_class(ds2QuirkDevice)
    .write_attr_button(
        attribute_name = "present_value",
        attribute_value = CALIB_TIME,
        cluster_id = AnalogInput.cluster_id,
        endpoint_id = 2,
        fallback_name = "Calibrate",
        translation_key = "calibrate",
    )
    .number(
        attribute_name = "present_value",
        cluster_id = AnalogInput.cluster_id,
        endpoint_id = 2,
        reporting_config=ReportingConfig(
            min_interval=0, max_interval=3600, reportable_change=1
        ),
        fallback_name = "Calibration time",
        device_class = NumberDeviceClass.DURATION,
        mode = NumberMode.BOX,
        max_value = CALIB_TIME
    )
    .replace_cluster_occurrences(m2MultistateValue)
    .add_to_registry()
)


