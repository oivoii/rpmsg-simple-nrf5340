# This check is needed to avoid infinite recursion. This module code will
# be executed for all images in the build, also for the child image being
# added below.
if (CONFIG_SECURE_BOOT)
  if (CONFIG_SOC_NRF5340_CPUNET AND !CONFIG_IS_SECURE_BOOTLOADER)
    add_child_image(
      NAME b0n
      SOURCE_DIR  ${NRF_DIR}/samples/nrf5340/netboot
    )
  endif()
endif()

if (CONFIG_INCLUDE_NET_IMAGE)
  set(CHILD_IMAGE_NAME "net_core")
  set(CHILD_IMAGE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../net")

  # Only include a child image if both image name and path have been defined.
  # Prevents e.g. menuconfig from becoming inaccessible, among others.
  if(CHILD_IMAGE_NAME AND CHILD_IMAGE_PATH)

    if (CONFIG_NRF53_UPGRADE_NETWORK_CORE)
      # Inject this configuration from parent image to enable
      # secure bootloader on the network core. This enables firmware update
      # of the network core application.
      add_overlay_config(
        ${CHILD_IMAGE_NAME}
        "${NRF_DIR}/subsys/bootloader/image/secure_boot.conf"
      )
    endif()

    add_child_image(
      NAME ${CHILD_IMAGE_NAME}
      SOURCE_DIR ${CHILD_IMAGE_PATH}
      DOMAIN "CPUNET"
      BOARD ${CONFIG_DOMAIN_CPUNET_BOARD}
      )
  endif()
endif()
