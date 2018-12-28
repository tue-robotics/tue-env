# -----------------------------------------
# Fix annoying perl language warnings
export LC_ALL="C.UTF-8"

# ------------------------------------------
# pip bash completion
if hash pip 2> /dev/null
then
	_pip_completion()
	{
	    COMPREPLY=( $( COMP_WORDS="${COMP_WORDS[*]}" \
	                   COMP_CWORD=$COMP_CWORD \
	                   PIP_AUTO_COMPLETE=1 $1 ) )
	}
	complete -o default -F _pip_completion pip
fi	
