#!/usr/bin/env sh

#
# build.sh
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

set -euo pipefail

cyan="\033[36m"
green="\033[32m"
bright_red="\033[91m"
reset="\033[0m"

green_esc=$(printf $green)
bright_red_esc=$(printf $bright_red)
reset_esc=$(printf $reset)

#
# Analyze Packages
#
mypy_opts="--check-untyped-defs"

echo "> Analyzing packages ..."

packages=(pure_pursuit scripts)

for package in "${packages[@]}"; do
    echo "  > ${cyan}$package${reset} ..."
    mypy -p $package $mypy_opts | sed -e "s|^|    |" -e "s|Success|${green_esc}success${reset_esc}|" -e "s|error:|${bright_red_esc}error${reset_esc}:|"
done

echo "\n> Build successful ${green}✔${reset}"
