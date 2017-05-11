---
-
  name: prod
  resources:
    -
      prefix: v3
      files:
        - piksi_firmware_v3_prod.stripped.elf
  artifacts:
    -
      bucket: swiftnav-releases
      prefix: piksi_buildroot/M4_BUILDROOT_VERSION/v3/prod
      files:
        - uImage.piksiv3_prod
    -
      bucket: swiftnav-releases
      prefix: piksi_fpga/M4_FPGA_VERSION
      files:
        - piksi_prod_fpga.bit
