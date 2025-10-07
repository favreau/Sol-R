#!/bin/bash
# Debug script for tsdViewer with Sol-R ANARI device

export LD_LIBRARY_PATH=/home/cfavreau/code/favreau/VisRTX/build/Debug:/home/cfavreau/code/favreau/Sol-R/build/Debug/devices/solr:$LD_LIBRARY_PATH
export TSD_ANARI_LIBRARIES=solr
export ANARI_LIBRARY_solr=/home/cfavreau/code/favreau/Sol-R/build/Debug/devices/solr/libanari_library_solr.so

echo "Environment variables set:"
echo "  LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo "  TSD_ANARI_LIBRARIES=$TSD_ANARI_LIBRARIES"
echo "  ANARI_LIBRARY_solr=$ANARI_LIBRARY_solr"
echo ""
echo "Starting GDB..."

gdb -ex "set environment LD_LIBRARY_PATH $LD_LIBRARY_PATH" \
    -ex "set environment TSD_ANARI_LIBRARIES solr" \
    -ex "set environment ANARI_LIBRARY_solr $ANARI_LIBRARY_solr" \
    /home/cfavreau/code/favreau/VisRTX/build/Debug/tsdViewer

