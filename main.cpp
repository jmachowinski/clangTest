#include <iostream>
#include <chrono>
#include <clang-c/Index.h>

using namespace std::chrono;

int main(int argc, char **argv) {
    
    CXIndex idx = clang_createIndex(0, 1);
    
    CXTranslationUnit TU;
    
    unsigned int options = CXTranslationUnit_DetailedPreprocessingRecord
//                            | CXTranslationUnit_Incomplete
                            | CXTranslationUnit_PrecompiledPreamble
                            | CXTranslationUnit_CacheCompletionResults
                            | CXTranslationUnit_CreatePreambleOnFirstParse;

    high_resolution_clock::time_point start = high_resolution_clock::now();
    
    std::cout << "Parse start" << std::endl;
    
    clang_parseTranslationUnit2(idx, "../test/tdot_asguard.cpp", argv, argc, nullptr, 0, options, &TU);

    high_resolution_clock::time_point end = high_resolution_clock::now();
    std::cout << "Parse took " << duration_cast<duration<double>>(end-start).count() << std::endl;

    start = high_resolution_clock::now();
    clang_reparseTranslationUnit(TU, 0, nullptr, CXReparse_None);
    end = high_resolution_clock::now();
    std::cout << "ReParse took " << duration_cast<duration<double>>(end-start).count() << std::endl;
    start = high_resolution_clock::now();
    clang_reparseTranslationUnit(TU, 0, nullptr, CXReparse_None);
    end = high_resolution_clock::now();
    std::cout << "ReParse took " << duration_cast<duration<double>>(end-start).count() << std::endl;
    
    return 0;
}
