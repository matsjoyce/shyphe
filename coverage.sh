#!/bin/bash
python3 test.py --coverage
lcov --capture --directory . --output-file .coverage.info --no-external
genhtml .coverage.info --output-directory coveragehtml
rm .coverage.info
