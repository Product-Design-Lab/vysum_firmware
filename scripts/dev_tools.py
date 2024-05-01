import argparse
import json
import os
import sys
import logging

# Set up basic configuration for logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

CONFIG_FILENAME = "config.json"
DEFAULT_CONFIG = {
    "name": "PDL",
    "email": "name@example.com",
    "lib_path": os.path.join("..", "libraries"),
}

def init_config():
    if os.path.exists(CONFIG_FILENAME):
        logging.info("Config file already exists")
        return

    try:
        with open(CONFIG_FILENAME, "w") as f:
            json.dump(DEFAULT_CONFIG, f, indent=4)
        logging.info("Config file created")
    except IOError as e:
        logging.error(f"Failed to create config file: {e}")

def _set_config(key, val):
    if not os.path.exists(CONFIG_FILENAME):
        logging.error("Config file does not exist")
        return

    try:
        with open(CONFIG_FILENAME, "r") as f:
            config = json.load(f)
    except (IOError, json.JSONDecodeError) as e:
        logging.error(f"Failed to read the config file: {e}")
        return

    config[key] = val

    try:
        with open(CONFIG_FILENAME, "w") as f:
            json.dump(config, f, indent=4)
        logging.info(f"Config updated: {key} = {val}")
    except IOError as e:
        logging.error(f"Failed to write to the config file: {e}")

def create_new_library(name):
    if not isinstance(name, str):
        logging.error("Library name must be a string")
        return

    try:
        with open(CONFIG_FILENAME, "r") as f:
            config = json.load(f)
    except FileNotFoundError:
        logging.error("Config file does not exist")
        return
    except json.JSONDecodeError:
        logging.error("Config file is not valid JSON")
        return

    if "lib_path" not in config:
        logging.error("lib_path not set in config")
        return

    lib_path = config["lib_path"]
    new_library_path = os.path.join(lib_path, name)
    if os.path.exists(new_library_path):
        logging.error("Library with the same name already exists")
        return

    try:
        os.makedirs(os.path.join(new_library_path, "src"))
        os.makedirs(os.path.join(new_library_path, "example", "demo"))
    except OSError as e:
        logging.error(f"Failed to create directories: {e}")
        return

    file_structure = {
        os.path.join("src", f"{name}.h"): '#pragma once\n\n',
        os.path.join("src", f"{name}.cpp"): f'#include "{name}.h"\n\n',
        os.path.join("example", "demo", "demo.ino"): 
            '#include <Arduino.h>\n#include <{0}.h>\n\nvoid setup() \n{{\n\n}}\nvoid loop() \n{{\n\n}}\n'.format(name),
        "library.properties": 
            f'name={name}\nversion=0.0.0\nauthor={config["name"]}\nmaintainer={config["name"]} <{config["email"]}>\n'
            'sentence=Short description of the library\nparagraph=Longer description of the library\ncategory=\nurl=http://example.com\n',
        "keywords.txt": f'{name} KEYWORD1\n',
        "README.md": f'# {name}\n\n',
        "LICENSE": 'License text\n'
    }

    for file_path, content in file_structure.items():
        try:
            with open(os.path.join(new_library_path, file_path), "w") as f:
                f.write(content)
        except IOError as e:
            logging.error(f"Failed to write file {file_path}: {e}")

    logging.info(f"Library created at {new_library_path}")

def config_name(name):
    _set_config("name", name)

def config_email(email):
    _set_config("email", email)

def config_lib_path(path):
    _set_config("lib_path", path)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PDL Development Tools")
    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("init_config", help="Initialize the config file")
    library_parser = subparsers.add_parser("create_new_library", help="Initialize a new library")
    library_parser.add_argument("name", help="Name of the library")

    config_parser = subparsers.add_parser("config", help="Set a configuration value")
    config_parser.add_argument("key", choices=["name", "email", "lib_path"])
    config_parser.add_argument("value")

    args = parser.parse_args()

    if args.command == "init_config":
        init_config()
    elif args.command == "create_new_library":
        create_new_library(args.name)
    elif args.command == "config":
        if args.key == "name":
            config_name(args.value)
        elif args.key == "email":
            config_email(args.value)
        elif args.key == "lib_path":
            config_lib_path(args.value)
    else:
        parser.print_help()
