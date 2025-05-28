#!/bin/bash

HOME_DOCKERDEV=$(getent passwd dockerdev | cut -d: -f6)
sudo chown -R dockerdev:dockerdev $HOME_DOCKERDEV

exec "$@"
