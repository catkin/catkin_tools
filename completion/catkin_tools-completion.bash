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

_catkin_last_option()
{
  # search backwards for the last given option
  for (( i=${cword} ; i > 0 ; i-- )) ; do
    if [[ ${words[i]} == -* ]]; then
      echo ${words[i]}
      return
    fi
  done
}

_catkin_verb()
{
  # search forwards to find catkin verb (neglecting global catkin opts)
  for (( i=1 ; i < ${cword} ; i++ )) ; do
    if [[ ${words[i]} == -* ]] ; then continue; fi
    if [[ ${catkin_verbs} == *${words[i]}* ]] ; then
      echo ${words[i]}
      return
    fi
  done
}

_catkin_pkgs()
{
  # return list of all packages
  catkin --no-color list --unformatted --quiet 2> /dev/null
}

# TODO:
# - parse --workspace and --profile options in order to complete outside of cwd

_catkin()
{
  local cur prev words cword catkin_verbs catkin_opts
  _init_completion || return # this handles default completion (variables, redirection)

  # complete to the following verbs
  local catkin_verbs="build clean config create init list profile run_tests"

  # filter for long options (from bash_completion)
  local OPTS_FILTER='s/.*\(--[-A-Za-z0-9]\{1,\}=\{0,1\}\).*/\1/p'

  # complete to verbs ifany of these are the previous word
  local catkin_opts=$(catkin --help 2>&1 | sed -ne $OPTS_FILTER | sort -u)

  # complete catkin profile subcommands
  local catkin_profile_args="add list remove rename set"

  local verb=$(_catkin_verb)
  case ${verb} in
    "")
      if [[ ${cur} == -* ]]; then
        COMPREPLY=($(compgen -W "${catkin_opts}" -- ${cur}))
      else
        COMPREPLY=($(compgen -W "${catkin_verbs}" -- ${cur}))
      fi
      ;;
    build)
      if [[ ${cur} == -* ]]; then
        local catkin_build_opts=$(catkin build --help 2>&1 | sed -ne $OPTS_FILTER | sort -u)
        COMPREPLY=($(compgen -W "${catkin_build_opts}" -- ${cur}))
      else
        COMPREPLY=($(compgen -W "$(_catkin_pkgs)" -- ${cur}))
      fi
      ;;
    config)
      # list all options
      local catkin_config_opts=$(catkin config --help 2>&1 | sed -ne $OPTS_FILTER | sort -u)
      COMPREPLY=($(compgen -W "${catkin_config_opts}" -- ${cur}))

      # list package names when --whitelist or --blacklist was given as last option
      if [[ ${cur} != -* && $(_catkin_last_option) == --*list ]] ; then
        COMPREPLY+=($(compgen -W "$(_catkin_pkgs)" -- ${cur}))
      fi

      # list directory names when useful
      if [[ ${prev} == --extend || ${prev} == --*-space ]] ; then
        # add directory completion
        compopt -o nospace 2>/dev/null
        COMPREPLY+=($(compgen -d -S "/" -- ${cur}))
      fi
      ;;
    clean)
      local catkin_clean_opts=$(catkin clean --help 2>&1 | sed -ne $OPTS_FILTER | sort -u)
      COMPREPLY=($(compgen -W "${catkin_clean_opts}" -- ${cur}))
      ;;
    create)
      if [[ "${words[@]}" == *" pkg"* ]] ; then
        local catkin_create_pkg_opts=$(catkin create pkg --help 2>&1 | sed -ne $OPTS_FILTER | sort -u)
        COMPREPLY=($(compgen -W "${catkin_create_pkg_opts}" -- ${cur}))
      else
        COMPREPLY=($(compgen -W "pkg" -- ${cur}))
      fi
      ;;
    profile)
      case ${prev} in
        profile)
          COMPREPLY=($(compgen -W "${catkin_profile_args}" -- ${cur}))
          ;;
        set|rename|remove)
          COMPREPLY=($(compgen -W "$(catkin --no-color profile list --unformatted)" -- ${cur}))
          ;;
        *)
          COMPREPLY=()
          ;;
      esac
      ;;
    init)
      local catkin_init_opts=$(catkin init --help 2>&1 | sed -ne $OPTS_FILTER | sort -u)
      COMPREPLY=($(compgen -W "${catkin_init_opts}" -- ${cur}))
      ;;
    list)
      local catkin_list_opts=$(catkin list --help 2>&1 | sed -ne $OPTS_FILTER | sort -u)
      COMPREPLY=($(compgen -W "${catkin_list_opts}" -- ${cur}))
      ;;
    run_tests)
      if [[ ${cur} == -* ]]; then
        local catkin_run_tests_opts=$(catkin run_tests --help 2>&1 | sed -ne $OPTS_FILTER | sort -u)
        COMPREPLY=($(compgen -W "${catkin_run_tests_opts}" -- ${cur}))
      else
        COMPREPLY=($(compgen -W "$(_catkin_pkgs)" -- ${cur}))
      fi
      ;;
  esac

  return 0
}

complete -F _catkin catkin
