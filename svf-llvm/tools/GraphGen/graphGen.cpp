//===- wpa.cpp -- Whole program analysis -------------------------------------//
//
//                     SVF: Static Value-Flow Analysis
//
// Copyright (C) <2013-2017>  <Yulei Sui>
//

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.

// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//===-----------------------------------------------------------------------===//

/*
 // Whole Program Pointer Analysis
 //
 // Author: Yulei Sui,
 */

#include "Graphs/IVFG.h"
#include "Graphs/PEG.h"
#include "SVF-LLVM/LLVMUtil.h"
#include "SVF-LLVM/SVFIRBuilder.h"
#include "Util/CommandLine.h"
#include "Util/Options.h"
#include "WPA/Andersen.h"

using namespace llvm;
using namespace std;
using namespace SVF;

static Option<bool> PEGGen("peg", "Generate PEG", false);
static Option<bool> VFGGen("vfg", "Generate VFG", false);

int main(int argc, char** argv)
{

    char** arg_value = new char*[argc];
    std::vector<std::string> moduleNameVec;
    moduleNameVec =
        OptionBase::parseOptions(argc, argv, "Whole Program Points-to Analysis",
                                 "[options] <input-bitcode...>");

    if (Options::WriteAnder() == "ir_annotator")
    {
        LLVMModuleSet::getLLVMModuleSet()->preProcessBCs(moduleNameVec);
    }

    SVFModule* svfModule =
        LLVMModuleSet::getLLVMModuleSet()->buildSVFModule(moduleNameVec);

    /// Build SVFIR
    SVFIRBuilder builder(svfModule);
    SVFIR* pag = builder.build();

    if (PEGGen())
    {
        PEG* graph = new PEG();
        graph->build(pag);
        graph->writeGraph("peg.g");
    }
    else if (VFGGen())
    {
        AndersenWaveDiff* ander = AndersenWaveDiff::createAndersenWaveDiff(pag);
        auto memSSA = new SaberSVFGBuilder();
        memSSA->buildFullSVFG(ander);
        IVFG* graph = new IVFG();
        graph->build(memSSA->getSVFG());
        graph->writeGraph("vfg.g");
    }

    delete[] arg_value;
    return 0;
}
