# Catkin Shell Verbs

function __catkin_help_augmented() {
  # Print out an augmented --help line

  # Get catkin executable
  CATKIN=$1

  # Add our shell verbs to the main helpline
  ORIGINAL_HELP_TEXT="$($CATKIN --help)"
  AUGMENTED_HELP_TEXT=""

  # Read each line of the help text, preserving whitespace
  while IFS= read -r LINE; do
    if [[ $LINE =~ "^\s+clean\s+" ]]; then
      # Insert cd summary
      AUGMENTED_HELP_TEXT+="$CATKIN_CD_SUMMARY\n$LINE\n"
    elif [[ $LINE =~ "^\s+profile\s+" ]]; then
      # Insert source summary
      AUGMENTED_HELP_TEXT+="$LINE\n$CATKIN_SOURCE_SUMMARY\n"
    elif [[ $LINE =~ "^catkin command" ]]; then
      # Update summary
      AUGMENTED_HELP_TEXT+="catkin command with shell verbs\n"
    elif [[ $LINE =~ "^\s+\[\s*[a-z]+\s*\|{0,1}" ]]; then
      # Update verb list
      VERB_LIST=${LINE/build/build | cd}
      VERB_LIST=${VERB_LIST/profile/source | profile}
      VERB_LIST="$(echo "$VERB_LIST" | fmt -c -w 80)"
      AUGMENTED_HELP_TEXT+="$VERB_LIST\n"
    else
      # Pass-though
      AUGMENTED_HELP_TEXT+="$LINE\n"
    fi
  done <<< "$ORIGINAL_HELP_TEXT"
  echo $AUGMENTED_HELP_TEXT
}

function catkin() {
  # Define help lines
  CATKIN_CD_SUMMARY='    cd   	Changes directory to a package or space.'
  CATKIN_SOURCE_SUMMARY='    source      Sources a resultspace environment.'

  # Get actual catkin executable
  # Using `command` we ignore shell functions
  CATKIN="$(command which catkin)"

  # Get setup file extension
  if [ -n "$ZSH_VERSION" ]; then
    SHELL_EXT="zsh"
  elif [ -n "$BASH_VERSION" ]; then
    SHELL_EXT="bash"
  else
    SHELL_EXT=""
  fi

  # Capture original args
  ORIG_ARGS=(${@[*]})

  # Handle main arguments
  OPTSPEC=":hw-:"
  WORKSPACE_ARGS=""

  # Process main arguments
  while getopts "$OPTSPEC" optchar ; do
    case "${optchar}" in
      -)
        case "${OPTARG}" in
          # TODO: replace --args below with `$1` ?
          workspace) WORKSPACE_ARGS="--workspace $2"; OPTIND=$(( $OPTIND + 1 ));;
          profile) PROFILE_ARGS="--profile $2"; OPTIND=$(( $OPTIND + 1 ));;
          help) __catkin_help_augmented $CATKIN; return;;
        esac;;
      w) WORKSPACE_ARGS="--workspace $2";;
      h) __catkin_help_augmented $CATKIN; return;;
      *);;
    esac
  done

  MAIN_ARGS="${WORKSPACE_ARGS} ${PROFILE_ARGS}"

  # Get subcommand
  SUBCOMMAND="$1"

  # Check if there's no subcommand
  if [ -z "$SUBCOMMAND" ]; then
    __catkin_help_augmented $CATKIN; return
  fi

  # Shift subcommand
  shift

  # Handle shell verbs
  case "${SUBCOMMAND}" in
    cd) cd "$($CATKIN locate $MAIN_ARGS $@)";;
    source) source "$($CATKIN locate $MAIN_ARGS -d)/setup.$SHELL_EXT";;
    *) $CATKIN ${ORIG_ARGS}
  esac
}
