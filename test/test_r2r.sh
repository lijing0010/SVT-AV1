para_count=1200
enc_modes="6 4 2"
#qp_sets="63 55 43 32 20"
qp_sets="63 43 20"

test_specific_file=0 #Whether use specific file as source 
specific_y4m_file=~/tmp/clips/netflix_drivingpov_65f.y4m

#Or use all clips or part of random files within a directory
y4m_dir=~/tmp/clips/

#If using directory, just test up to $total_file_count files, in case it contains too many files
#total_file_count=1
total_file_count=0

if [ ${test_specific_file} -eq 1 ]; then
    source_y4ms=${specific_y4m_file}
else
    if [ $total_file_count -eq 0 ]; then
        source_y4ms="${y4m_dir}/*"
    else
        source_y4ms=`shuf -e ${y4m_dir}/* |head -n ${total_file_count}`
    fi
fi


function clean_up_files() {
    rm *.yuv *.bin *.log log*.txt recon*.txt input*.txt core > /dev/null 2>&1
}

function check_bitstream_r2r() {
    has_r2r=0
    first=1
    firstname=""
    md5_val=""
    for i in `ls out*.bin`
    do
        if [ ${first} -eq 1 ]; then
            first=0
            firstname=${i}
            md5_val=`md5sum $i|cut -d " " -f 1`
            continue
        fi

        cmp --silent ${firstname} ${i}
        if [ $? -eq 0 ]; then
            :
            #echo "No problem "
            #rm ${out_bin} ${recon} ${decode}
        else
            has_r2r=1
            break
        fi
    done

    if [ ${has_r2r} -eq 1 ]; then
        echo "        *EncMode $2 has R2R, dump all the md5sum"
        return 0
    else
        #echo "        Finished ${para_count} instances on file $1, no r2r for M$2, md5sum is ${md5_val}"
        echo "        Finished ${para_count} instances, no R2R, md5sum is ${md5_val}"
        return 1
    fi
}

function check_r2r_with_encmode() {
    #$1 as enc mode
#    for y4m in ${y4m_dir}/*
    for y4m in ${source_y4ms}
    do
        for qp in ${qp_sets}
        do
            echo "    Processing file ${y4m} on QP ${qp}..."
            clean_up_files
            seq -w ${para_count} | parallel --bar -j 100 ./SvtAv1EncApp -i ${y4m} -n 65 -intra-period 72 -lad 0 -lp 1 -q ${qp} -enc-mode $1 -b out{}.bin  "&>" log{}.txt
            if check_bitstream_r2r ${y4m} $1
            then
                return $?
            fi
        done
    done
    return 1
}

for em in ${enc_modes}
do
    echo "Testing M${em}"
    if check_r2r_with_encmode ${em}
    then
        exit
    fi
done

exit

has_r2r=0
first=1
firstname=""
md5_val=""
for i in `ls out*.bin`
do
    if [ ${first} -eq 1 ]; then
        first=0
        firstname=${i}
        md5_val=`md5sum $i|cut -d " " -f 1`
        continue
    fi

    cmp --silent ${firstname} ${i}
    if [ $? -eq 0 ]; then
        :
        #echo "No problem "
        #rm ${out_bin} ${recon} ${decode}
    else
        has_r2r=1
        break
    fi
done

if [ ${has_r2r} -eq 0 ]; then
    echo "        Running ${para_count} instances, no r2r for encmode ${encmode}, md5sum is ${md5_val}"
    rm out*.bin #log*.txt
    exit
else
    md5sum *.bin
    echo "        *EncMode ${encmode} has R2R, dump all the md5sum"
fi

exit
log_diff=0
first_log=1
first_log_name=""
for i in `ls log*`
do
    if [ ${first_log} -eq 1 ]; then
        first_log=0
        first_log_name=${i}
        md5_val=`md5sum $i|cut -d " " -f 1`
        continue
    fi

    cmp --silent ${first_log_name} ${i}
    if [ $? -eq 0 ]; then
        :
        #echo "No problem "
        #rm ${out_bin} ${recon} ${decode}
    else
        log_diff=1
        break
    fi
done

if [ ${log_diff} -eq 0 ]; then
    echo "logs are the same, next round"
else
    md5sum log*
    echo "Check the details!!!"
fi

#    val=`md5sum $i|cut -d " " -f 1`
#    if [ $val == ${m4_result} ]; then
#        #echo $i is OK
#        rm $i
#    else
#        echo $i is different
#    fi
