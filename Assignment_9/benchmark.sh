COMMAND=${1:-./reference_simulation}
COMMANDS=("$COMMAND" "./reference_simulation")
RUNS=${2:-5}
TIME=${3:-3}
COUNTERS=(0 0)
COUNTER=0
TOTALS=(0 0)
echo "Comparing $COMMAND to reference over $RUNS runs at $TIME seconds each..."
while [[ $COUNTER -lt $(($RUNS * 2)) ]]; do
	if [ $((COUNTER % 2)) -eq 0 ]
		then
			echo "Run $((${COUNTERS[$((COUNTER % 2))]}+1))"
	fi
	PARSE="$(timeout $TIME ${COMMANDS[$((COUNTER % 2))]} | grep 'Particles Per Second' | cut -c 21-)"
	INNER_COUNTER=0
	SUBTOTAL=0
	RESULT=($PARSE)
	END=$((${#RESULT[@]} - 1))
	for ((i=$END;i > $(($END - 10));i--)); do
		RAW=${RESULT[i]}
		NUM=$(printf "%.0f\n", $RAW)
	 	NUM=${NUM%??}
	 	let SUBTOTAL=SUBTOTAL+NUM
	 	let INNER_COUNTER=INNER_COUNTER+1
	done
	AVERAGE=$((SUBTOTAL / INNER_COUNTER))
	echo -e " ${COMMANDS[$((COUNTER % 2))]}\t Avg. PPS: $AVERAGE"
	TOTALS[$((COUNTER % 2))]=$((${TOTALS[$((COUNTER % 2))]}+$AVERAGE))
	COUNTERS[$((COUNTER % 2))]=$((${COUNTERS[$((COUNTER % 2))]}+1))
	let COUNTER=COUNTER+1
done
echo ""
echo -e "${COMMANDS[0]}\t Avg. PPS: $((${TOTALS[0]} / ${COUNTERS[0]}))"
echo -e "${COMMANDS[1]}\t Avg. PPS: $((${TOTALS[1]} / ${COUNTERS[1]}))"