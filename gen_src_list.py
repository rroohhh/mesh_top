#!/usr/bin/env python3

import sys
from pathlib import Path
from collections import defaultdict

class TY:
    class SV: ...
    class VHDL: ...
    class V: ...

def process_sourcelists(flavor, files):
    for file in files:
        cwd = Path(file).parent
        for line in Path(file).read_text().splitlines():
            line, *comment = line.strip().split('#')
            if ":" in line:
                cond, line = line.split(':')
                line = line.strip()
                if cond != flavor:
                    continue
            line = line.strip()
            if len(line) == 0:
                continue
            for file in cwd.glob(line):
                match file.suffix:
                    case ".vhd" | ".vhdl":
                        ty = TY.VHDL
                    case ".sv":
                        ty = TY.SV
                    case ".v":
                        ty = TY.V
                yield ty, str(file)

def gen_yosys(gen):
    by_type = defaultdict(list)
    for ty, file in gen:
        by_type[ty].append(str(Path(file).absolute()))

    script = []
    for ty, files in by_type.items():
        match ty:
            case TY.SV:
                script += [f"!sv2v -E always -E assert -E logic -E unbasedunsized {' '.join(files)} > /tmp/lrensalrtiensa"]
                script += [f"read -sv /tmp/lrensalrtiensa"]
            case TY.VHDL:
                script += [f"ghdl -Wno-pure -Wno-hide --std=08 -frelaxed  -fsynopsys {' '.join(files)} -e"]
            case TY.V:
                script += [f"read -sv {' '.join(files)}"]

    return "\n".join(script)



def gen_synopsys(gen):
    import zipfile
    with zipfile.ZipFile("gen/sources.zip", "w") as zip:
        script = ["read_file -autoread -top top [list \\"]
        for ty, file in gen:
            script.append(f"{file} \\")
            with zip.open(file, "w") as f:
                f.write(Path(file).read_bytes())
            # match ty:
            #     case TY.SV:
            #     case TY.VHDL:
            #         script.append(f"analyze -format vhdl {file}")
            #     case TY.V:
            #         script.append(f"analyze -format verilog {file}")

        res = "\n".join(script) + "\n ]"
        with zip.open("read_src.tcl", "w") as f:
            f.write(res.encode())
        return res



if __name__ == "__main__":
    flavor, *files = sys.argv[1:]

    match flavor:
        case "yosys":
            print(gen_yosys(process_sourcelists(flavor, files)))
        case "synopsys":
            print(gen_synopsys(process_sourcelists(flavor, files)))
        case _:
            assert False, "unknown flavor"
