import os
import sys
import unittest
import time
from datetime import datetime

import yaml
import boto3
from botocore.handlers import disable_signing

from upload_artifacts import (
  upload_firmware,
  cp_fpga_firmware,
  cp_buildroot_images,

)

TEST_BUCKET=os.environ.get('TEST_BUCKET')


class TestUploadArtifacts(unittest.TestCase):
  '''Sanity tests. Note that test-build.yml uses the actual buckets
  for the fpga and sd card images. Should only be reading from these. 
  Write bucket (firmware_key_prefix) should have anonymous write permissions.'''

  def setUp(self):
    with open('test-build.yml', 'r') as f:
      self.build_config = yaml.load(f)
    timestamp = datetime.strftime(datetime.utcnow(),'%Y-%m-%dT%T')
    self.build_name = timestamp + '_42_abcdefg'
    self.firmware_key_prefix = '{0}/{1}/{2}'.format(
      self.build_config['piksi_version'],
      'master',
      self.build_name
    )
    self.s3 = boto3.resource('s3')

  def test_upload_firmware(self):
    self.s3.meta.client.meta.events.register('choose-signer.s3.*', disable_signing)
    firmware_key = self.firmware_key_prefix + '/piksi_firmware.hex'
    # so the folders don't have the same timestamp
    time.sleep(1)
    
    upload_firmware(self.build_config, firmware_key, self.s3)

    bucket = self.s3.Bucket(TEST_BUCKET)
    results = bucket.objects.filter(Prefix='v3/master/' + self.build_name + '/piksi_firmware.hex')
    self.assertEqual(len(list(results)), 1)

  def test_cp_fgpa_firmware(self):
    # so the folders don't have the same timestamp
    time.sleep(1)

    cp_fpga_firmware(self.build_config, self.s3, self.firmware_key_prefix)
    
    bucket = self.s3.Bucket(TEST_BUCKET)
    results = bucket.objects.filter(Prefix='v3/master/' + self.build_name + '/piksi_microzed_fpga_v3.5_unlocked.bit')
    self.assertEqual(len(list(results)), 1)

  def test_cp_buildroot_images(self):
    cp_buildroot_images(self.build_config, self.s3, self.firmware_key_prefix)
    
    bucket = self.s3.Bucket(TEST_BUCKET)
    results = bucket.objects.filter(Prefix='v3/master/' + self.build_name)
    self.assertEqual(len(list(results)), 4)


if __name__ == "__main__":
  # install boto3 and yaml, probably in a virtualenv
  if not TEST_BUCKET:
    print "Environment variable TEST_BUCKET must be set"
    sys.exit(-1)
  unittest.main()
