#include <iostream>
using namespace std;

int main()
{
    // Config config = Config::load("config/planner_config.json");

    GridMap map = MapLoader::load("assets/map01.txt", config.map);
    // Pose start{...};
    // Pose goal{...};

    // HybridAStar planner(config.planner);
    // planner.setMap(map);
    // planner.setStart(start);
    // planner.setGoal(goal);

    // PlanResult result = planner.plan();

    // JsonExporter::save(result, "output/result.json");
    // SvgExporter::save(result, map, "output/result.svg");
}
