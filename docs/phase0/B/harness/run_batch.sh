#!/bin/bash
# Usage: run_batch.sh <custom|ifra> <outdir>   (sim must be running with that plugin)
H=$(cd "$(dirname "$0")" && pwd); source $H/env.sh; C=$1; O=$2
T="timeout 400 python3 $H/attach_test.py run --candidate $C --outdir $O"
$T --runs 5 --tag ${C}_main > $O/${C}_main.txt 2>&1; echo "main rc=$?"
$T --runs 2 --tag ${C}_fast075 --fast 0.75 > $O/${C}_fast.txt 2>&1; echo "fast rc=$?"
$T --runs 1 --tag ${C}_offset --offset --double_attach > $O/${C}_offset.txt 2>&1; echo "offset rc=$?"
$T --runs 1 --tag ${C}_reattach --reattach > $O/${C}_reattach.txt 2>&1; echo "reattach rc=$?"
