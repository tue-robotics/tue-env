#!/usr/bin/env bash

authorsFile="$TUE_DIR/installer/targets/amigo-user/scripts/git-authors.txt"
                                                                             
IFS=$'\n' authorsStrAll=($(cat $authorsFile))                                    
rm gitsu.txt
                                                                             
for authorStr in ${authorsStrAll[@]}                                             
do                                                                               
	echo $authorStr                                                              
	                                                                             
	IFS='=' authorsStrArray=($authorStr)                                         
	authorNick=$(echo ${authorsStrArray[0]} | sed -e 's/^ *//' -e 's/ *$//')     
	echo $authorNick                                                             
	                                                                             
	IFS=' ' authorsStrArray=(${authorsStrArray[1]})                              
	authorName=$(echo ${authorsStrArray[@]:0:2} | sed -e 's/^ *//' -e 's/ *$//') 
	echo $authorName                                                             
	authorEmail=$(echo ${authorsStrArray[2]} | sed -e 's/^ *//' -e 's/ *$//' -e 's/^<*//' -e 's/>*$//')
	echo $authorEmail
	
	echo '-------------'                                                         

    echo "$authorEmail : $authorName" >> gitsu.txt
done
