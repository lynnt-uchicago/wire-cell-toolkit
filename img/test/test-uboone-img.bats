#!/usr/bin/env bats

# Initial test of generating files for input to possible GNNs for
# charge solving and deghosting tasks.

# Note: this test may break for a while as we consolidate the testing
# system and test data repo.

@test "generate gnn data" {

    # FIXME: after consolidation, use wcb-bats.sh library.  For now,
    # user must add the directory holding data files to WIRECELL_PATH.
    celltree=""
    for maybe in $(echo ${WIRECELL_PATH} | tr ":" "\n")
    do
        if [ -f "${maybe}/celltreeOVERLAY.root" ] ; then
            celltree="${maybe}/celltreeOVERLAY.root"
            break
        fi
    done
    [[ -n "$celltree" ]]
    [[ -f "$celltree" ]]

    run wire-cell -A celltree="$celltree" -c test-gnn-fodder.jsonnet
    echo "$output"
    [[ "$status" -eq 0 ]]

    #### to be continued ####
}
