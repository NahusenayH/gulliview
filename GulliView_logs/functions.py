# Copyright © 2025 Emil Nylander

# This file is part of GulliView logs.

# GulliView logs is free software: you can redistribute it and/or 
# modify it under the terms of the GNU General Public License 
# as published by the Free Software Foundation, either version 3 
# of the License, or (at your option) any later version.

# GulliView logs is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
# General Public License for more details.

# You should have received a copy of the GNU General Public License 
# along with GulliView logs. If not, see <https://www.gnu.org/licenses/>.

#%% Standard modules
from tabulate import tabulate

#%% Command handler
def run_command(commands, menupath = "path: start"):

    if type(commands) == type(list()):
        commands[0]()
        commands = commands[1]
    
    options = list(commands.keys())

    print("","-"*20)
    print(menupath)
    print("leave empty to return/close")

    option = input_str(options)
    
    #If empty, return to parent
    if option == "":
        return True # This breaks main loop
    
    #If dict, it is a submenu
    if type(commands[option]) == type(dict()):
        run_command(commands[option], menupath + "/" + option)
    
    #If array, it is a submenu with a function before
    elif type(commands[option]) == type(list()):
        commands[option][0]()
        run_command(commands[option][1], menupath + "/" + option, 
                    commands[option][0])
    
    #Else it is a function
    else:
        commands[option]()
    
# Only allows commands in allowedInputs and returns the input
# If noneAllowed is true then empty strings can be returned
def input_str(allowed_inputs, exclude = [], noneAllowed = True, number = True):

    # title
    input_list = f"\ninput number to choose:"

    # add optios to title so it becomes one long string
    for i in range(len(allowed_inputs)):

        # If number is True add number or X before each line
        if number and allowed_inputs[i] not in exclude:
            indent = f"  {i+1}) "
        elif number:
            indent = "  -) "

        # If number is false we remove line if exclude
        elif i not in exclude:
            indent = " "*5
        else:
            continue
        
        # add option to string
        input_list = f"{input_list}\n{indent}{allowed_inputs[i]}"
    
    # add 2 linebreaks
    input_list = input_list + "\n\n"
    
    while True:
        
        option = input(input_list)

        if noneAllowed and option == "":
            return ""

        # Check if number
        try:
            option = int(option)
        except:
            print("\n-----not a number, try again-----\n")
            continue
        
        if option > len(allowed_inputs) or option <= 0:
            print("\n-----number not in range, try again-----\n")
            continue
        
        # covert to corresponding string
        option = allowed_inputs[option-1]
        
        # check if in exclude
        if option in exclude:
            print("\n-----already selected, try again-----\n")
            continue
        
        return option

# Handles input of integer from 0 to "high"
def input_int(high, prompt, allow_zero = False):
    if allow_zero:
        lower_limit = 0
    else:
        lower_limit = 1

    while True:
        userInput = input(prompt)
        if userInput == "":
            return None
        try:
            number = int(userInput)
        except:
            print("-----invalid number-----")
            continue
        
        if number > high or number < lower_limit:
            print("-----number not in range-----")
            continue
        
        break
    return number

# Press enter to continue
def user_acknowledge(msg):
    print(msg)
    print("Press enter to continue...")
    input()  # Waits for user input before proceeding

# Returns dict as a string looking like a table
def tabulate_dict(data, headers = [], tablefmt = 'rounded_grid'):
    array = []
    for key in data:
        array.append([key, data[key]])
    return tabulate(array, headers, tablefmt=tablefmt)

def try_int_float_convert(value):
    try:
        if "." in value:
            return float(value)
        else:
            return int(value)
    except ValueError:
        return value