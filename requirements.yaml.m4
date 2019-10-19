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
      bucket: M4_BUCKET
      prefix: piksi_buildroot_private/M4_BUILDROOT_VERSION/v3/prod
      files:
        - uImage.piksiv3_prod
    -
      bucket: M4_BUCKET
      prefix: piksi_fpga/M4_FPGA_VERSION
      files:
        - piksi_prod_fpga.bit
-
  name: base
  resources:
    -
      prefix: v3
      files:
        - piksi_firmware_v3_base.stripped.elf
  artifacts:
    -
      bucket: M4_BUCKET
      prefix: piksi_buildroot_private/M4_BUILDROOT_VERSION/v3/prod
      files:
        - uImage.piksiv3_prod
    -
      bucket: M4_BUCKET
      prefix: piksi_fpga/M4_FPGA_VERSION
      files:
        - piksi_base_fpga.bit
