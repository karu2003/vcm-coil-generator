# VCM Coil Generator — KiCad Plugin
# Entry point: registers ActionPlugin with KiCad

from .vcm_coil_action import VCMCoilPlugin

VCMCoilPlugin().register()
