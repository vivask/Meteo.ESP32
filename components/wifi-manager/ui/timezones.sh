#!/bin/bash

cat ./timezones.csv | awk -F\" '{print "<option value=\""$4"\">"$2"</option>"}' > ./timezones.html