#!/usr/bin/env bash
set -eu -o pipefail

function exit_if_new_paths_not_exist()
{
  if [[ ! -d "${new_src_dir}" ]] # if path not exist
  then
    echo "Error: Directory ${new_src_dir} does not exists." 
    exit 1
  fi

  if [[ ! -d "${new_gui_build_dir}" ]] # if path not exist
  then
    echo "Error: Directory ${new_gui_build_dir} does not exists."
    exit 1
  fi
}

function warn_if_current_paths_not_exist()
{
  local prompt="false"

  if [[ ! -d "${current_src_dir}" ]] # if path not exist
  then
    echo "Warning: Directory ${current_src_dir} does not already exists."
    prompt="true"
  fi

  if [[ ! -d "${current_gui_build_dir}" ]] # if path not exist
  then
    echo "Warning: Directory ${current_gui_build_dir} does not already exists."
    prompt="true"
  fi

  if [[ $prompt == "true" ]]
  then
    read -p 'See warning above. Do you want to proceed? (y/n)' yn
    if [[ "${yn}" == "y" ]]
    then
      echo "Proceeding..."
    else
      echo "Exiting"
      exit 0
    fi
  fi
}

script_dir=$(dirname -- $0) # path of the directory containing this script
new_src_dir="${script_dir}/../stride_ws/src"
new_gui_build_dir="${script_dir}/../stride_gui_build"
current_src_dir="${HOME}/stride_ros/stride_ws/src"
current_gui_build_dir="${HOME}/stride_ros/stride_gui_build"

exit_if_new_paths_not_exist
warn_if_current_paths_not_exist

# stride_ws
rm -rf "${current_src_dir}"
mkdir -p "${current_src_dir}"
cp -r "${new_src_dir}"/* "${current_src_dir}"

# stride_gui
rm -rf "${current_gui_build_dir}"
mkdir -p "${current_gui_build_dir}"
cp -r "${new_gui_build_dir}"/* "${current_gui_build_dir}"

echo "Update Completed"