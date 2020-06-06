goal = (6, 5)
def h(n):
    a,b = n
    x, y = goal
    return √( (a-x)^2 + (b-y)^2)

def astar1(start):
    visited = {} # Map node => cost)
    q = []
    q.push( (0, h(start), [], start) )

    for (pathcost, estimate, path, node) in pop_smallest_estimate(q):
        if node in visited and visited(node) < h(node):
                continue
        else:
            visited[node] = estimate

        if is_goal(node):
            return (path + [node], cost)

        for (child, cost) in children(node):
            q.push((
                pathcost + cost,
                pathcost + cost + h(child),
                path + [node],
                node
            ))

    return None

def pop_smallest_estimate(list):
    sort(list, key= lambda elem: elem[1])

    return list.pop()

