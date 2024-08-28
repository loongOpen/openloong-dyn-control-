#!/bin/bash

# Define the header text
header_text="/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#"

# Function to add header to files
add_header_to_files() {
    for file in *.h *.cpp; do
        if [[ -f $file ]]; then
            # Determine the comment symbol based on file extension
            if [[ $file == *.c || $file == *.h ]]; then
                comment_symbol="//"
            elif [[ $file == *.cpp ]]; then
                comment_symbol="//"
            fi

            # Add header to the beginning of the file
            echo "$header_text" | cat - "$file" >tempfile && mv tempfile "$file"

            # Remove content before the first '#'
            sed -i "0,/^#/s///" "$file"
        fi
    done
}
