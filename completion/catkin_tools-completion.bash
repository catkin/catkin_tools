# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ZSH support
if [[ -n ${ZSH_VERSION-} ]]; then
  autoload -U +X bashcompinit && bashcompinit
fi

_last_option()
{
  # search backwards for the last given option
  for (( i=${COMP_CWORD} ; i > 0 ; i-- )) ; do
    if [[ ${COMP_WORDS[i]} == -* ]]; then
      echo ${COMP_WORDS[i]}
      return
    fi
  done
}

# TODO:
# - parse --workspace and --profile options in order to complete outside of cwd
# - autocomplete build options

_catkin()
{
  local cur prev catkin_verbs catkin_opts

  COMPREPLY=()

  cur=${COMP_WORDS[COMP_CWORD]}
  prev=${COMP_WORDS[COMP_CWORD-1]}

  # complete to the following verbs
  catkin_verbs="\
build   - Build packages
clean   - Clean workspace components
config  - Configure workspace 
create  - Create workspace components
init    - Initialize workspace
list    - List workspace components
profile - Switch between configurations"

  # complete to verbs ifany of these are the previous word
  catkin_opts="\
--force-color
--no-color
--test-colors"

  # complete popular catkin build options
  catkin_build_opts="\
--help
--dry-run
--this
--no-deps
--unbuilt
--start-with-this
--continue-on-failure
--force-cmake
--pre-clean
--get-env
--verbose
--interleave-output
--no-status
--summarize
--no-notify
--env-cache
--no-env-cache"

  # complete popular catkin clean options
  catkin_clean_opts="
--help
--all
--build
--devel
--install
--cmake-cache
--setup-files
--orphans"

  # complete popular catkin config options
  catkin_config_opts="--help
--init
--extend
--no-extend
--install
--no-install
--whitelist
--blacklist
--no-whitelist
--no-blacklist
--cmake-args
--make-args
--catkin-make-args
--space-suffix
--merge-devel
--link-devel
--isolate-devel"

  # complete popular catkin create options
  catkin_create_pkg_opts="\
--help
--version
--license
--maintainer
--author
--description
--catkin-deps
--system-deps
--boost-components"

  # complete catkin profile subcommands
  catkin_profile_args="\
add
list
remove
rename
set"

  local OLDIFS="$IFS"
  local IFS=$'\n'

  if [[ ${COMP_CWORD} -eq 1 || ${catkin_opts} == *${prev}* ]] ; then
    COMPREPLY=($(compgen -W "${catkin_verbs}" -- ${cur}))
  else
    if [[ "${COMP_WORDS[@]}" == *" build"* ]] ; then
      if [[ ${cur} == -* ]]; then
        COMPREPLY=($(compgen -W "${catkin_build_opts}" -- ${cur}))
      else
        COMPREPLY=($(compgen -W "$(catkin --no-color list --unformatted --quiet)" -- ${cur}))
      fi
    elif [[ "${COMP_WORDS[@]}" == *" config"* ]] ; then
      if [[ ${cur} != -* && "--whitelist --blacklist" == *$(_last_option)* ]] ; then
        COMPREPLY=($(compgen -W "$(catkin --no-color list --unformatted --quiet)" -- ${cur}))
      else
        COMPREPLY=($(compgen -W "${catkin_config_opts}" -- ${cur}))
      fi
    elif [[ "${COMP_WORDS[@]}" == *" clean"* ]] ; then
      COMPREPLY=($(compgen -W "${catkin_clean_opts}" -- ${cur}))
    elif [[ "${COMP_WORDS[@]}" == *" create"* ]] ; then
      if [[ "${COMP_WORDS[@]}" == *" pkg"* ]] ; then
        COMPREPLY=($(compgen -W "${catkin_create_pkg_opts}" -- ${cur}))
      else
        COMPREPLY=($(compgen -W "pkg" -- ${cur}))
      fi
    elif [[ "${COMP_WORDS[@]}" == *" profile"* ]] ; then
      case ${prev} in
        profile)
          COMPREPLY=($(compgen -W "${catkin_profile_args}" -- ${cur}))
          ;;
        set)
          COMPREPLY=($(compgen -W "$(catkin --no-color profile list --unformatted)" -- ${cur}))
          ;;
        rename)
          COMPREPLY=($(compgen -W "$(catkin --no-color profile list --unformatted)" -- ${cur}))
          ;;
        remove)
          COMPREPLY=($(compgen -W "$(catkin --no-color profile list --unformatted)" -- ${cur}))
          ;;
        *)
          COMPREPLY=()
          ;;
      esac
    else
      case ${prev} in
        init)
          COMPREPLY=()
          ;;
        list)
          COMPREPLY=()
          ;;
        *)
          COMPREPLY=()
          ;;
      esac
    fi
  fi

  IFS="$OLDIFS"
  if [[ ${#COMPREPLY[*]} -eq 1 ]]; then #Only one completion
    COMPREPLY=( ${COMPREPLY[0]%% - *} ) #Remove ' - ' and everything after
  fi

  return 0
}

complete -F _catkin catkin
