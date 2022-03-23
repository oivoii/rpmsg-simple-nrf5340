# This check is needed to avoid infinite recursion. This module code will
# be executed for all images in the build, also for the child image being
# added below.
if (CONFIG_INCLUDE_NET_IMAGE)
  add_child_image(
    NAME net_core
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../net
    DOMAIN cpunet
    BOARD nrf5340dk_nrf5340_cpunet
    )
endif()
