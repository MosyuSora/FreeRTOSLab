import argparse
import os
import shutil
from jinja2 import Environment, FileSystemLoader

def createTraceConfigFile(name, path, psram):
    env = Environment(loader=FileSystemLoader("templates/"))
    template = env.get_template("traceConfig.h.j2")

    # Prepare the context for the Jinja2 template.
    context = {
        "name": name,
        "projectPath": path,
        "psram": psram
    }
    
    content = template.render(context)

    filename = os.path.join(path, "traceConfig.h")

    with open(filename, mode="w", encoding="utf-8") as cmakeLists:
        cmakeLists.write(content)

    print(os.path.abspath(filename))


def createFreeRTOSConfigFile(name, path, trace):
    env = Environment(loader=FileSystemLoader("templates/"))
    template = env.get_template("FreeRTOSConfig.h.j2")

    # Prepare the context for the Jinja2 template.
    context = {
        "name": name,
        "projectPath": path,
        "trace": trace
    }
    
    content = template.render(context)

    filename = os.path.join(path, "FreeRTOSConfig.h")

    with open(filename, mode="w", encoding="utf-8") as cmakeLists:
        cmakeLists.write(content)

    print(os.path.abspath(filename))

def createCMakeFile(name, path, noRTOS, trace):
    env = Environment(loader=FileSystemLoader("templates/"))
    template = env.get_template("CMakeLists.txt.j2")

    # Prepare the context for the Jinja2 template.
    context = {
        "name": name,
        "projectPath": path,
        "noRTOS": noRTOS,
        "trace": trace
    }
    
    content = template.render(context)

    filename = os.path.join(path, "CMakeLists.txt")

    with open(filename, mode="w", encoding="utf-8") as cmakeLists:
        cmakeLists.write(content)

    print(os.path.abspath(filename))

def createMainFile(name, path, noRTOS, trace):
    env = Environment(loader=FileSystemLoader("templates/"))
    template = env.get_template("main.c.j2")

    # Prepare the context for the Jinja2 template.
    context = {
        "name": name,
        "projectPath": path,
        "noRTOS": noRTOS,
        "trace": trace
    }
    
    content = template.render(context)

    filename = os.path.join(path, "main.c")

    with open(filename, mode="w", encoding="utf-8") as cmakeLists:
        cmakeLists.write(content)

    print(os.path.abspath(filename))

def printOptionHelper(option, value):
    if value:
        print("[x] " + option)
    else:
        print("[ ] " + option)

def createProject(name, path, noRTOS, trace, psram):
    
    # Print the confiuguration to the console
    print("Creating Project: " + name)
    printOptionHelper("FreeRTOS enabled", not noRTOS)
    printOptionHelper("Tracing enabled", trace)
    printOptionHelper("Tracing buffer in PSRAM", psram)
    print("------------------------------------")

    # Create project specific files from templates
    createCMakeFile(name, path, noRTOS, trace)
    createMainFile(name, path, noRTOS, trace)
    
    # Copy other needed files to project
    if not noRTOS:
        createFreeRTOSConfigFile(name, path, trace)

        if trace:
            createTraceConfigFile(name, path, psram)

        kernelImportSrc = os.path.join("static", "FreeRTOS_Kernel_import.cmake")
        kernelImportDst = os.path.join(path, "FreeRTOS_Kernel_import.cmake")
        shutil.copyfile(kernelImportSrc, kernelImportDst)
        print(os.path.abspath(kernelImportDst))
    
    sdkImportSrc = os.path.join("static", "pico_sdk_import.cmake")
    sdkImportDst = os.path.join(path, "pico_sdk_import.cmake")
    shutil.copyfile(sdkImportSrc, sdkImportDst)
    print(os.path.abspath(sdkImportDst))

def main():
    parser = argparse.ArgumentParser(
                    prog='ProjectCreator',
                    description='Tool to help setup new projects for the ES Lab-Kit.')
    parser.add_argument("projectName", help="Name of the new project.", type = str)
    parser.add_argument('-noRTOS', help="Add option for a bare metal project.", action='store_true')
    parser.add_argument('-trace', help="Set option to include traceing infrastructure and store trace in local SRAM.", action='store_true')
    parser.add_argument('-tracePSRAM', help="Set option to include traceing infrastructure and store trace in PSRAM.", action='store_true')

    args = parser.parse_args()

    if (args.trace or args.tracePSRAM) and args.noRTOS:
        print("Error: Trace can only be enables with FreeRTOS enabled.")
        return 1

    if args.trace and args.tracePSRAM:
        print("Error: trace and tracePSRAM are set. Only one option is allowed at a time!")
        return 1
    
    if args.trace or args.tracePSRAM:
        traceEnabled = True
    else:
        traceEnabled = False

    
    # Create the path for the new project. All projects are stored in the Software directory.
    projectPath = os.path.join("..", "..")
    projectPath = os.path.join(projectPath, "Software")
    projectPath = os.path.join(projectPath, "Projects")
    projectPath = os.path.join(projectPath, args.projectName)
    
    # Abort if the project directory exists already, else create the directory
    if os.path.isdir(projectPath):
        print("Error: Project folder exists!")
        return 1
    os.makedirs(projectPath)

    # Create the new project
    createProject(args.projectName, projectPath, args.noRTOS, traceEnabled, args.tracePSRAM)

if __name__ == "__main__":
    main()