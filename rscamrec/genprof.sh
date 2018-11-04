gprof ./align gmon.out |gprof2dot > aa.dot
dot -T png aa.dot > aa.png
lcov --capture --output-file debug.info
genhtml debug.info --output-directory output