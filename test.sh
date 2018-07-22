revision=`git rev-parse HEAD`
for i in `seq 0 6`; do
    ./main.py --test $i --headless
    a=$?
    echo "Ref $revision test $i > result $a" >> testlog
done
