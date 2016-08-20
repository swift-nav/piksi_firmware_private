import os
import sys
import unittest
from datetime import datetime

import yaml
import boto3
from botocore.handlers import disable_signing

from upload_artifacts import (
  upload_firmware,
  cp_fpga_firmware,
  cp_buildroot_images,

)


class TestUploadArtifacts(unittest.TestCase):
  '''Just sanity tests. Note that test-build.yml uses the actual buckets
  for the fpga and sd card images. Should only be reading from these. 
  Write bucket is margaret-test which has anonymous-write permissions.'''

  def setUp(self):
    with open('test-build.yml', 'r') as f:
      self.build_config = yaml.load(f)

  def test_upload_firmware(self):
    s3 = boto3.resource('s3')
    s3.meta.client.meta.events.register('choose-signer.s3.*', disable_signing)
    timestamp = 'UTC' + datetime.strftime(datetime.utcnow(),'%Y-%m-%d-%T')
    build_name = timestamp + '_TRAVIS-1_COMMIT-a1b2c4d/piksi_firmware.hex'
    firmware_key = '{0}/{1}/{2}'.format(
      self.build_config['piksi_version'],
      'master',
      build_name
    )

    upload_firmware(self.build_config, firmware_key, s3)
    
    bucket = s3.Bucket('margaret-test')
    results = bucket.objects.filter(Prefix='v3/master/' + build_name)
    self.assertEqual(len(list(results)), 1)

  def test_cp_fgpa_firmware(self):
    s3 = boto3.resource('s3')
    timestamp = 'UTC' + datetime.strftime(datetime.utcnow(),'%Y-%m-%d-%T')
    build_name = timestamp + '_TRAVIS-1_COMMIT-a1b2c4d'
    firmware_key = '{0}/{1}/{2}'.format(
      self.build_config['piksi_version'],
      'master',
      build_name
    )

    cp_fpga_firmware(self.build_config, s3, firmware_key)
    
    bucket = s3.Bucket('margaret-test')
    results = bucket.objects.filter(Prefix='v3/master/' + build_name + '/piksi_microzed_fpga_v3.5_unlocked.bit')
    self.assertEqual(len(list(results)), 1)
    

  def test_cp_buildroot_images(self):
    s3 = boto3.resource('s3')
    timestamp = 'UTC' + datetime.strftime(datetime.utcnow(),'%Y-%m-%d-%T')
    build_name = timestamp + '_TRAVIS-1_COMMIT-a1b2c4d'
    firmware_key = '{0}/{1}/{2}'.format(
      self.build_config['piksi_version'],
      'master',
      build_name
    )

    cp_buildroot_images(self.build_config, s3, firmware_key)
    
    bucket = s3.Bucket('margaret-test')
    results = bucket.objects.filter(Prefix='v3/master/' + build_name)
    for file in results:
      print file


if __name__ == "__main__":
  # install boto3 and yaml, probably in a virtualenv
  unittest.main()