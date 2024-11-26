#!/usr/bin/env sh

height=$1
packet_len=3
prefix=~/tmp/out/

for packet_len in $(seq 1 5); do
    for e in $(seq 0 0.1 0.95); do
        max_p=$(python -c "print((1 - ${e}) / (${packet_len} * (${height} - 1) * 4))")
        max_p_div_10=$(python -c "print(${max_p} / 10)")
        for p in $(seq ${max_p_div_10} ${max_p_div_10} ${max_p}); do
            n=height_${height}_p_${p}_e_${e}_plen_${packet_len}
            echo ${n}
            ./sim  --config_rate ${p} --event_rate ${e} --height ${height} --width 1 --out ${prefix}/${n}.fst --packet_len 1 --rng_seed 123 --steps 1000000 > ${prefix}/${n}.log
        done
    done
done
