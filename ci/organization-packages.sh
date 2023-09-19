#! /usr/bin/env bash

for i in "$@"
do
    case $i in
        -o=* | --organization=* )
            ORGANIZATION="${i#*=}"
            ;;
        -t=* | --type=* )
            PACKAGE_TYPE="${i#*=}"
            ;;
        --token=* )
            TOKEN="${i#*=}"
            ;;
        * )
            # unknown option
            if [[ -n "${i}" ]]  # Ignore empty arguments
            then
                echo -e "\e[35m\e[1mUnknown input argument '${i}'. Check CI .yml file\e[0m"
                exit 1
            fi
            ;;
    esac
    shift
done

echo -e "\e[35m\e[1mORGANIZATION = ${ORGANIZATION}\e[0m"
echo -e "\e[35m\e[1mPACKAGE_TYPE = ${PACKAGE_TYPE}\e[0m"
echo -e "\e[35m\e[1mTOKEN length = ${#TOKEN}\e[0m"

echo "::group::curl -L -H \"Accept: application/vnd.github+json\" -H \"Authorization: Bearer \${TOKEN}\" -H \"X-GitHub-Api-Version: 2022-11-28\" \"https://api.github.com/orgs/${ORGANIZATION}/packages?package_type=${PACKAGE_TYPE}&per_page=100&page=1\""
echo -e "\e[1mcurl -L -H \"Accept: application/vnd.github+json\" -H \"Authorization: Bearer \${TOKEN}\" -H \"X-GitHub-Api-Version: 2022-11-28\" \"https://api.github.com/orgs/${ORGANIZATION}/packages?package_type=${PACKAGE_TYPE}&per_page=100&page=1\""
output=$(curl -L -H "Accept: application/vnd.github+json" -H "Authorization: Bearer ${TOKEN}" -H "X-GitHub-Api-Version: 2022-11-28" "https://api.github.com/orgs/${ORGANIZATION}/packages?package_type=${PACKAGE_TYPE}&per_page=100&page=1")
echo -e "${output}" | jq -C
echo "::endgroup::"

echo -e "\e[1mjq -r '.[].name'\e[0m"
packages=$(jq -r '.[].name' <<< "${output}")
echo -e "\e[35m\e[1mpackages:\e[0m"
for pkg in ${packages}
do
    echo -e "\e[35m\e[1m  ${pkg}\e[0m"
    packages_list="${packages_list:+$packages_list, }'${pkg}'"
done
packages_list="[${packages_list}]"
