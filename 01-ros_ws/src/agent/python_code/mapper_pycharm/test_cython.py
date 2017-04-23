from planning.heuristic import kinodyn_metric, saturate_params

import time
import numpy.random as rnd
import numpy as np

# Parameters
n_data = 1000
n_queries = 30
n = 6 # dimension of the state vector
X = [(0,100),(0,100),(10,20),(-10,10),(-10,10),(-1,1)] # x,y,a,\dot{x},\dot{y},\dot{a}
U = [(-2,2),(-2,2),(-.2,.2)] # \dotdot{x},\dotdot{y},\dotdot{a}
rnd.seed(10)
dt = 1.
k_nearest = 10
k_reduced = n_data/5

# init
data = []
queries = []

for i in range(n_data):
    data.append([x1 + (x2 - x1) * rnd.rand() for (x1, x2) in X])
for i in range(n_queries):
    queries.append([x1 + (x2 - x1) * rnd.rand() for (x1, x2) in X])
data = np.array(data, dtype='float32')
queries = np.array(queries, dtype='float32')

start = time.time()
for i in range(n_queries):
    x_new, q_params = saturate_params(data[i], queries[i], X, U, 0., dt)
end = time.time()
print 'time to compute',n_queries,'extensions',end - start

############################################
print ''
print 'look up test with',n,'dimensions,',n_data,'points in the database/base and',n_queries,'queries'
import numpy as np
from scipy.spatial.distance import cdist
from time import time
import mrpt

# Generate synthetic test data
queries = queries

# Solve exact nearest neighbors with standard methods from scipy and numpy for reference
exact_search_time = time()
exact_neighbors = np.zeros((n_queries, k_nearest))
for i in range(n_queries):
    exact_neighbors[i] = np.argsort(cdist([queries[i]], data))[0,:k_nearest]
exact_search_time = time() - exact_search_time

# Offline phase: Indexing the data. This might take some time.
indexing_time = time()
index = mrpt.MRPTIndex(data, depth=5, n_trees=10)
index.build()
indexing_time = time() - indexing_time

# Online phase: Finding nearest neighbors stupendously fast.
approximate_search_time = time()
approximate_neighbors = np.zeros((n_queries, k_nearest))
for i in range(n_queries):
    approximate_neighbors[i] = index.ann(queries[i], k_nearest, votes_required=4)
approximate_search_time = time() - approximate_search_time


# Print some stats
print ('Indexing time: %1.3f seconds' %indexing_time)
print ('%d approximate queries time: %1.3f seconds' %(n_queries, approximate_search_time))
print ('%d exact queries time: %1.3f seconds' %(n_queries, exact_search_time))

correct_neighbors = 0
for i in range(n_queries):
    correct_neighbors += len(np.intersect1d(exact_neighbors[i], approximate_neighbors[i]))
print ('Average recall: %1.2f.' %(float(correct_neighbors)/(n_queries*k_nearest)))

#############################
result_baseline = []
search_time_start = time()
for i in range(n_queries):
    query = queries[i]

    shortest_dist = np.Inf
    selected_node = None
    for x in data:
        dist = kinodyn_metric(x, query, X, U)

        if dist <= shortest_dist:
            selected_node = x
            shortest_dist = dist
    result_baseline.append(selected_node)

search_time_start_baseline = time() - search_time_start
print ''
print 'data set points',n_data
print 'baseline time n_queries',n_queries,'in', search_time_start_baseline,'s'

result_reduced = []
search_time_start = time()
for i in range(n_queries):
    query = queries[i]

    # firstly, use a quick heuristic to get the k-closest neighbours (test showed that the MRPT is really slow)
    closestNeighboursIdx = np.argsort(cdist([query], data))[0, :k_reduced]
    reduced_data = data[closestNeighboursIdx]

    # secondly, use slow distance metric as proper metric
    shortest_dist = np.Inf
    selected_node = None
    for x in reduced_data:
        dist = kinodyn_metric(x, query, X, U)

        if dist <= shortest_dist:
            selected_node = x
            shortest_dist = dist
    result_reduced.append(selected_node)
search_time_start_reduced = time() - search_time_start
print 'reduced time n_queries',n_queries,'in', search_time_start_reduced,'s'
print 'speedup',round(search_time_start_baseline/search_time_start_reduced,1)

# accuracy
accu = 0.
error_displacement = 0.
relative_error = 0.
for i in range(n_queries):
    if np.array_equal(result_reduced[i],result_baseline[i]):
        accu += 1./n_queries
    error_displacement += np.linalg.norm(result_reduced[i] - result_baseline[i])
    true_dist = np.linalg.norm(queries[i]-result_baseline[i])
    estm_dist = np.linalg.norm(queries[i]-result_reduced[i])
    relative_error += (np.abs(estm_dist-true_dist)/true_dist)
print 'exact accuracy',round(accu,2)
print 'avr error displacement',round(error_displacement / n_queries, 2)
print 'avr relative error',round(relative_error / n_queries, 2)



#####

