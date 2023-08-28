#include <omp.h>
#include <stdio.h>

/*
This script is to test the nested parallelization using omp.  Compile in terminal using:

    $ g++ examples/cube_franka/parallelization_test.cc -fopenmp

Run using:

    $ ./a.out
*/

int main(void ){

    // This omp_get_max_threads() somehow always just is the total number of threads on the computer and doesn't seem to change based
    // on which are available.  On lab desktop, this is 12 threads.
    printf("Max threads now: %d \n", omp_get_max_threads());

    omp_set_nested(1);  // This is necessary to allow for nested parallelization.  Without this, the default is 0 which means it will
                        // only parallelize the outer loop and no additional inner loops.

    #pragma omp parallel for num_threads(9)     // Set this to the total number of threads that everything below will be using.  Every i
                                                // in the below loop will have this total number of threads divided by 3 (so 3 threads)
                                                // which it can use for future parallelization.
    for(int i=0; i< 3; i++) {
        int outer_num_threads = omp_get_num_threads();  // This returns the total number of threads used by the outer parallelization loop.
                                                        // This will be 9, as set by line 23.
        int outer_max = omp_get_max_threads();          // This is always the number of threads on the computer (12 on lab desktop).
        int outer_in_parallel = omp_in_parallel();      // This is a boolean flag; should be 1 to indicate omp nested parallelization is enabled.
        int outer_thread_id = omp_get_thread_num();     // This is the thread ID which is unique only to this outer parallel loop.  It will start
                                                        // at 0, and in this case go up to 2.

        #pragma omp parallel for num_threads(3)         // The `num_threads(3)` isn't necessary; it seems to work the same without it.
                                                        // Would need to be careful if trying to set this to something higher than the number of
                                                        // threads available to this process.  In this case, that's 3.
        for (int j=0; j< 3; j++) {
            int inner_num_threads = omp_get_num_threads();  // This returns the total number of threads used by the inner parallelization loop.
                                                            // This will be 3, as set by line 34.
            int inner_max = omp_get_max_threads();          // This is still always the number of threads on the computer (12 on lab desktop).
            int inner_in_parallel = omp_in_parallel();      // Should still be 1 to indicate nesting is allowed.
            int inner_thread_id = omp_get_thread_num();     // This is the inner thread ID, which is unique only to the inner parallel loop.
                                                            // These will also start at 0 and in this case go up to 2.  While the outer and inner
                                                            // thread IDs are not unique, the pair (outer_thread_id, inner_thread_id) is unique.

            printf("i = %d, j = %d, outer threadId = %d, inner threadId = %d, outer threads = %d, inner threads = %d, outer max = %d, inner max = %d, outer parallel = %d, inner parallel = %d \n",
                i, j, outer_thread_id, inner_thread_id, outer_num_threads, inner_num_threads, outer_max, inner_max, outer_in_parallel,
                inner_in_parallel);
        }
    }

    return(0);
}