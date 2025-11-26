import yaml
import jsonschema
import json
import argparse

def validate_yaml(yaml_path, schema_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    with open(schema_path, 'r') as f:
        schema = json.load(f)
    
    try:
        jsonschema.validate(instance=data, schema=schema)
        print("YAML validation succeeded!")
        return True
    except jsonschema.exceptions.ValidationError as e:
        print("YAML validation failed!")
        print(e.message)
        return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Validate YAML file against JSON schema.")
    parser.add_argument('yaml_file', type=str, help='Path to the YAML file')
    parser.add_argument('schema_file', type=str, help='Path to the JSON schema file')
    args = parser.parse_args()
    validate_yaml(args.yaml_file, args.schema_file)

