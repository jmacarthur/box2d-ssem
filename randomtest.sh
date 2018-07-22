revision=`git rev-parse HEAD`
for i in `seq 1 20`; do
    ./main.py --randomtest 0 --seed $i --headless
    a=$?
    echo "Ref $revision random test $i > result $a" >> randomtestlog
done
