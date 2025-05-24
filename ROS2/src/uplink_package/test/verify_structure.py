import os
import json


current_dir = os.path.dirname(__file__)
json_path = os.path.join(current_dir, 'packet_structure.json')

with open(json_path, 'r') as f:
    data = json.load(f)


input_order = data["Input_Order"]
output_order_A = data["Output_Order_A"]
output_order_B = data["Output_Order_B"]
fields = data["Fields"]

print(f"input order length - {len(input_order)}")
print(f"output_A length - {len(output_order_A)}")
print(f"output_B length - {len(output_order_B)}")

for ele in output_order_A:
    if ele in input_order and ele in fields:
        print(f"{ele} — present in Input_Order and Fields")
    else:
        print(f"{ele} — MISSING")

for ele in output_order_B:
    if ele in input_order and ele in fields:
        print(f"{ele} — present in Input_Order and Fields")
    else:
        print(f"{ele} — MISSING")

