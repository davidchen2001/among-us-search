from game import pathfinding

#fn f(n)= g(n) + h(n)
#hn f(n)= h(n)

print(pathfinding("./Maps/Polus.csv", 4, "fn")) #Killer wins
#print(pathfinding("./Maps/Polus.csv", 4, "hn"))

print(pathfinding("./Maps/Skeld.csv", 4, "fn")) #Crewmates win
#print(pathfinding("./Maps/Skeld.csv", 4, "hn"))

print(pathfinding("./Maps/Test0.csv", 4, "fn")) #Crewmate wins
print(pathfinding("./Maps/Test0.csv", 4, "hn")) #Killer wins