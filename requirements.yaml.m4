---
-
  name: microzed
  resources:
    -
      prefix: v3
      files:
        - piksi_firmware_v3_microzed.stripped.elf
  artifacts:
    -
      bucket: swiftnav-artifacts
      prefix: piksi_buildroot/M4_BUILDROOT_VERSION/v3/microzed
      files:
        - uImage.piksiv3_microzed
    -
      bucket: swiftnav-artifacts
      prefix: piksi_fpga/M4_FPGA_VERSION
      files:
      - piksi_microzed_nt1065_fpga.bit
-
  name: prod
  resources:
    -
      prefix: v3
      files:
        - piksi_firmware_v3_prod.stripped.elf
  artifacts:
    -
      bucket: swiftnav-artifacts
      prefix: piksi_buildroot/M4_BUILDROOT_VERSION/v3/prod
      files:
        - uImage.piksiv3_prod
    -
      bucket: swiftnav-artifacts
      prefix: piksi_fpga/M4_FPGA_VERSION
      files:
        - piksi_prod_fpga.bit
