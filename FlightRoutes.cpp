#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <cmath>
#include <cstring>
using namespace std;

class Flight_M
{
public:
    class Airport
    {
    public:
        unordered_map<string, int> nbrs; 
    };
    static unordered_map<string, Airport> airports;
    Flight_M()
    {
        airports.clear();
    }
    int numAirports()
    {
        return airports.size();
    }
    bool containsAirport(string aname)
    {
        return airports.count(aname) > 0;
    }
    void addAirport(string aname)
    {
        Airport airport;
        airports[aname] = airport;
    }
    void removeAirport(string aname)
    {
        Airport airport = airports[aname];
        vector<string> keys;
        for (auto it = airport.nbrs.begin(); it != airport.nbrs.end(); it++)
        {
            keys.push_back(it->first);
        }
        for (string key : keys)
        {
            Airport nbrAirport = airports[key];
            nbrAirport.nbrs.erase(aname);
        }
        airports.erase(aname);
    }
    int numRoutes()
    {
        int count = 0;
        for (auto it = airports.begin(); it != airports.end(); it++)
        {
            Airport airport = it->second;
            count += airport.nbrs.size();
        }
        return count / 2;
    }
    bool containsRoute(string aname1, string aname2)
    {
        if (airports.count(aname1) == 0 || airports.count(aname2) == 0 || airports[aname1].nbrs.count(aname2) == 0)
        {
            return false;
        }
        return true;
    }
    void addRoute(string aname1, string aname2, int value)
    {
        if (airports.count(aname1) == 0 || airports.count(aname2) == 0 || airports[aname1].nbrs.count(aname2) > 0)
        {
            return;
        }
        airports[aname1].nbrs[aname2] = value;
        airports[aname2].nbrs[aname1] = value;
    }
    void removeRoute(string aname1, string aname2)
    {
        if (airports.count(aname1) == 0 || airports.count(aname2) == 0 || airports[aname1].nbrs.count(aname2) == 0)
        {
            return;
        }
        airports[aname1].nbrs.erase(aname2);
        airports[aname2].nbrs.erase(aname1);
    }
    void display_Map()
    {
        cout << "\t Flight Network Map" << endl;
        cout << "\t------------------" << endl;
        cout << "----------------------------------------------------" << endl;
        cout << endl;
        for (auto it = airports.begin(); it != airports.end(); it++)
        {
            string key = it->first;
            string str = key + " =>" + "\n";
            Airport airport = it->second;
            for (auto it2 = airport.nbrs.begin(); it2 != airport.nbrs.end(); it2++)
            {
                string nbr = it2->first;
                str += "\t" + nbr + "\t";
                if (nbr.length() < 16)
                    str += "\t";
                if (nbr.length() < 8)
                    str += "\t";
                str += to_string(it2->second) + "\n";
            }
            cout << str << endl;
        }
        cout << "\t------------------" << endl;
        cout << "---------------------------------------------------" << endl;
    }
    void display_Airports()
    {
        cout << endl;
        int i = 1;
        for (auto it = airports.begin(); it != airports.end(); it++)
        {
            cout << i << ". " << it->first << endl;
            i++;
        }
        cout << endl;
    }

    bool hasRoute(string aname1, string aname2, unordered_map<string, bool> &processed)
    {
        if (containsRoute(aname1, aname2))
        {
            return true;
        }
        processed[aname1] = true;
        Airport airport = airports[aname1];
        for (auto it = airport.nbrs.begin(); it != airport.nbrs.end(); it++)
        {
            string nbr = it->first;
            if (! processed.count(nbr))
            {
                if (hasRoute(nbr, aname2, processed))
                {
                    return true;
                }
            }
        }
        return false;
    }

    class DijkstraPair
    {
    public:
        string aname; 
         string psf;   
        int cost;     
        bool operator<(const DijkstraPair &o) const
        {
            return cost > o.cost;
        }
    };

    int dijkstra(string src, string des, bool nan)
    {
        int val = 0;
        vector<string> ans;
        unordered_map<string, DijkstraPair> map;
        priority_queue<DijkstraPair> pq;
        for (auto it = airports.begin(); it != airports.end(); it++)
        {
            DijkstraPair np;
            np.aname = it->first;
            np.cost = INT_MAX;
            if (it->first == src)
            {
                np.cost = 0;
                np.psf = it->first;
            }
            pq.push(np);
            map[it->first] = np;
        }

        while (!pq.empty())
        {
            DijkstraPair rp = pq.top();
            pq.pop();
            if (rp.aname == des)
            {
                val = rp.cost;
                break;
            }
            map.erase(rp.aname);
            ans.push_back(rp.aname);
            Airport airport = airports[rp.aname];
            for (auto it = airport.nbrs.begin(); it != airport.nbrs.end(); it++)
            {
                string nbr = it->first;
                if (map.count(nbr))
                {
                    int oc = map[nbr].cost;
                    Airport k = airports[rp.aname];
                    int nc;
                    if (nan)
                        nc = rp.cost + 120 + 40 * k.nbrs[nbr];
                    else
                        nc = rp.cost + k.nbrs[nbr];
                    if (nc < oc)
                    {
                        DijkstraPair gp = map[nbr];
                        gp.psf = rp.psf + nbr;
                        gp.cost = nc;
                        pq.push(gp);
                    }
                }
            }
        }
        return val;
    }

    class Pair
    {
    public:
        string aname; // Airport name
        string psf;   // Path so far
        int min_dis;  // Minimum distance
        int sec_time; // Minimum time
    };

    string Get_Minimum_Distance(string src, string dst)
    {
        int min = INT_MAX;
        string ans = "";
        unordered_map<string, bool> processed;
        deque<Pair> stack;

        Pair sp;
        sp.aname = src;
        sp.psf = src + "  ";
        sp.min_dis = 0;
        sp.sec_time = 0;

        stack.push_front(sp);

        while (!stack.empty())
        {
            Pair rp = stack.front();
            stack.pop_front();
            if (processed.count(rp.aname))
            {
                continue;
            }

            processed[rp.aname] = true;

            if (rp.aname == dst)
            {
                int temp = rp.min_dis;
                if (temp < min)
                {
                    ans = rp.psf;
                    min = temp;
                }
                continue;
            }
            Airport rpvtx = airports[rp.aname];
            for (auto it = rpvtx.nbrs.begin(); it != rpvtx.nbrs.end(); it++)
            {
                string nbr = it->first;
                if (!processed.count(nbr))
                {
                    Pair np;
                    np.aname = nbr;
                    np.psf = rp.psf + nbr + "  ";
                    np.min_dis = rp.min_dis + rpvtx.nbrs[nbr];
                    stack.push_front(np);
                }
            }
        }
        ans = ans + to_string(min);
        return ans;
    }

    string Get_Minimum_Time(string src, string dst)
{
    int min = INT_MAX;
    string ans = "";
    unordered_map<string, bool> processed;
    deque<Pair> stack;

    Pair sp;
    sp.aname = src;
    sp.psf = src + "  ";
    sp.min_dis = 0;
    sp.sec_time = 0;

    stack.push_front(sp);


    while (!stack.empty())
    {
        Pair rp = stack.front();
        stack.pop_front();
        if (processed.count(rp.aname))
        {
            continue;
        }

        processed[rp.aname] = true;

        if (rp.aname == dst)
        {
            int temp = rp.sec_time;
            if (temp < min)
            {
                ans = rp.psf;
                min = temp;
            }
            continue;
        }
        Airport rpvtx = airports[rp.aname];
        for (auto it = rpvtx.nbrs.begin(); it != rpvtx.nbrs.end(); it++)
        {
            string nbr = it->first;
            if (!processed.count(nbr))
            {
                Pair np;
                np.aname = nbr;
                np.psf = rp.psf + nbr + "  ";
                np.sec_time = rp.sec_time + 300; 
                stack.push_front(np);
            }
        }
    }
    double hours = ceil((double)min / 3600);
    ans = ans + to_string(hours);
    return ans;
}
    vector<string> get_Interchanges(string str)
    {
        vector<string> arr;
        string res[100];
        int k = 0;
        char *temp = new char[str.length() + 1];
        strcpy(temp, str.c_str());
        char *token = strtok(temp, "  ");

        while (token != NULL)
        {
            res[k++] = token;
            token = strtok(NULL, "  ");
        }

        arr.push_back(res[0]);
        int count = 0;
        for (int i = 1; i < k - 1; i++)
        {
            int index = res[i].find('~');
            string s = res[i].substr(index + 1);
            if (s.length() == 2)
            {
                string prev = res[i - 1].substr(res[i - 1].find('~') + 1);
                string next = res[i + 1].substr(res[i + 1].find('~') + 1);
                if (prev == next)
                {
                    arr.push_back(res[i]);
                }
                else
                {
                    arr.push_back(res[i] + " ==> " + res[i + 1]);
                    i++;
                    count++;
                }
            }
            else
            {
                arr.push_back(res[i]);
            }
        }
        arr.push_back(to_string(count));
        arr.push_back(res[k - 1]);
        return arr;
    }
};

unordered_map<string, Flight_M::Airport> Flight_M::airports;

void Create_Flight_Map(Flight_M &f)
{
    f.addAirport("IGI Airport~Delhi");
    f.addAirport("CSMI Airport~Mumbai");
    f.addAirport("KIA Airport~Bangalore");
    f.addAirport("MAA Airport~Chennai");
    f.addAirport("NSCB Airport~Kolkata");
    f.addAirport("RGIA Airport~Hyderabad");
    f.addAirport("COK Airport~Kochi");
    f.addAirport("SVPI Airport~Ahmedabad");
    f.addAirport("GOI Airport~Goa");
    f.addAirport("PNQ Airport~Pune");
    f.addAirport("TRV Airport~Thiruvananthapuram");
    f.addAirport("JAI Airport~Jaipur");
    f.addAirport("IXE Airport~Mangaluru");
    f.addAirport("BBI Airport~Bhubaneswar");
    f.addAirport("LGBI Airport~Guwahati");
    f.addAirport("IDR Airport~Indore");

    f.addRoute("IGI Airport~Delhi", "CSMI Airport~Mumbai", 1400);
    f.addRoute("CSMI Airport~Mumbai", "KIA Airport~Bangalore", 840);
    f.addRoute("KIA Airport~Bangalore", "MAA Airport~Chennai", 350);
    f.addRoute("MAA Airport~Chennai", "NSCB Airport~Kolkata", 1650);
    f.addRoute("NSCB Airport~Kolkata", "RGIA Airport~Hyderabad", 600);
    f.addRoute("RGIA Airport~Hyderabad", "COK Airport~Kochi", 700);
    f.addRoute("COK Airport~Kochi", "SVPI Airport~Ahmedabad", 800);
    f.addRoute("SVPI Airport~Ahmedabad", "GOI Airport~Goa", 650);
    f.addRoute("GOI Airport~Goa", "PNQ Airport~Pune", 190);
    f.addRoute("PNQ Airport~Pune", "IGI Airport~Delhi", 1500);
    f.addRoute("TRV Airport~Thiruvananthapuram", "COK Airport~Kochi", 200);
    f.addRoute("JAI Airport~Jaipur", "IGI Airport~Delhi", 280);
    f.addRoute("IXE Airport~Mangaluru", "KIA Airport~Bangalore", 200);
    f.addRoute("BBI Airport~Bhubaneswar", "NSCB Airport~Kolkata", 440);
    f.addRoute("LGBI Airport~Guwahati", "NSCB Airport~Kolkata", 1000);
    f.addRoute("IDR Airport~Indore", "SVPI Airport~Ahmedabad", 570);
}

void printCodelist()
{
    cout << "List of airports along with their codes:" << endl;
    int i = 1, j = 0, m = 1;
    string codes[Flight_M::airports.size()];
    for (auto it = Flight_M::airports.begin(); it != Flight_M::airports.end(); it++)
    {
        string key = it->first;
        codes[i - 1] = "";
        j = 0;
        for (int k = 0; k < key.length(); k++)
        {
            char c = key[k];
            if (isdigit(c))
            {
                codes[i - 1] += c;
                j++;
            }
            else if (isalpha(c))
            {
                codes[i - 1] += c;
            }
        }
        if (codes[i - 1].length() < 2)
            codes[i - 1] += toupper(key[1]);
        cout << i << ". " << key << "\t";
        if (key.length() < (22 - m))
            cout << "\t";
        if (key.length() < (14 - m))
            cout << "\t";
        if (key.length() < (6 - m))
            cout << "\t";
        cout << codes[i - 1] << endl;
        i++;
        if (i == pow(10, m))
            m++;
    }
}

int main()
{
    Flight_M f;
    Create_Flight_Map(f);
    cout << "\n\t\t\t*WELCOME TO THE FLIGHT APP" << endl;
    bool st=true;
    while (st)
    {
        cout << "\t\t\t\t~LIST OF ACTIONS~\n\n"
             << endl;
        cout << "1. LIST ALL THE AIRPORTS IN THE MAP" << endl;
        cout << "2. SHOW THE FLIGHT MAP" << endl;
        cout << "3. GET SHORTEST DISTANCE FROM A 'SOURCE' AIRPORT TO 'DESTINATION' AIRPORT" << endl;
        cout << "4. GET SHORTEST TIME TO REACH FROM A 'SOURCE' AIRPORT TO 'DESTINATION' AIRPORT" << endl;
        cout << "5. GET SHORTEST PATH (DISTANCE WISE) TO REACH FROM A 'SOURCE' AIRPORT TO 'DESTINATION' AIRPORT" << endl;
        cout << "6. GET FARE PRICE TO REACH FROM A 'SOURCE' AIRPORT TO 'DESTINATION' AIRPORT" << endl;
        cout << "7. EXIT THE MENU" << endl;
        cout << "\nENTER YOUR CHOICE FROM THE ABOVE LIST (1 to 7) : ";
        int choice = -1;
        cin >> choice;
        cout << "\n*\n"
             << endl;
        if (choice == 7)
        {
            break;
        }
        if (choice == 1)
        {
            f.display_Airports();
        }
        else if (choice == 2)
        {
            f.display_Map();
        }
        else if (choice == 3)
        {
            printCodelist();
            cout << "\n1. TO ENTER SERIAL NO. OF AIRPORTS\n2. TO ENTER CODE OF AIRPORTS\n"
                 << endl;
            cout << "ENTER YOUR CHOICE:";
            int ch;
            cin >> ch;
            int j;
            string st1 = "", st2 = "";
            cout << "ENTER THE SOURCE AND DESTINATION AIRPORTS" << endl;
            if (ch == 1)
            {
                int a, b;
                cin >> a >> b;
                vector<string> keys;
                for (auto it = Flight_M::airports.begin(); it != Flight_M::airports.end(); it++)
                {
                    keys.push_back(it->first);
                }
                st1 = keys[a - 1];
                st2 = keys[b - 1];
            }
            else if (ch == 2)
            {
                string a, b;
                cin >> a >> b;
                for (auto it = Flight_M::airports.begin(); it != Flight_M::airports.end(); it++)
                {
                    string key = it->first;
                    string code = "";
                    int j = 0;
                    for (int k = 0; k < key.length(); k++)
                    {
                        char c = key[k];
                        if (isdigit(c))
                        {
                            code += c;
                            j++;
                        }
                        else if (isalpha(c))
                        {
                            code += c;
                        }
                    }
                    if (code.length() < 2)
                        code += toupper(key[1]);
                    if (code == a)
                    {
                        st1 = key;
                    }
                    if (code == b)
                    {
                        st2 = key;
                    }
                }
            }
            else
            {
                cout << "Invalid choice" << endl;
                break;
            }
            unordered_map<string, bool> processed;
            if (!f.containsAirport(st1) || !f.containsAirport(st2) || !f.hasRoute(st1, st2, processed))
                cout << "THE INPUTS ARE INVALID" << endl;
            else
                cout << "SHORTEST DISTANCE FROM " << st1 << " TO " << st2 << " IS " << f.dijkstra(st1, st2, false) << " KM" << endl;
            break;
        }
        else if (choice == 4)
        {
            f.display_Airports();

            int srcIndex, desIndex;
            cout << "Enter the index of the source airport: ";
            cin >> srcIndex;

            cout << "Enter the index of the destination airport: ";
            cin >> desIndex;

            vector<string> airportNames;
            for (const auto &it : Flight_M::airports)
            {
                airportNames.push_back(it.first);
            }

            if (srcIndex >= 1 && srcIndex <= airportNames.size() &&
                desIndex >= 1 && desIndex <= airportNames.size())
            {
                string src = airportNames[srcIndex - 1];
                string des = airportNames[desIndex - 1];

                int shortestTime = f.dijkstra(src, des, true);

                if (shortestTime != numeric_limits<int>::max())
                {
                    cout << "Shortest time from " << src << " to " << des << " is " << shortestTime / 1500 << " minutes." << endl;
                }
                else
                {
                    cout << "No path found from " << src << " to " << des << "." << endl;
                }
            }
            else
            {
                cout << "Invalid airport indices." << endl;
            }
        }
        else if (choice == 5)
        {
            f.display_Airports();
            cout << "ENTER THE SOURCE AND DESTINATION AIRPORTS (by index)" << endl;
            int index1, index2;
            cin >> index1 >> index2;

            unordered_map<string, bool> processed2;
            vector<string> airportNames;
            for (const auto &it : Flight_M::airports)
            {
                airportNames.push_back(it.first);
            }

            if (index1 >= 1 && index1 <= airportNames.size() &&
                index2 >= 1 && index2 <= airportNames.size())
            {
                string s1 = airportNames[index1 - 1];
                string s2 = airportNames[index2 - 1];

                if (!f.containsAirport(s1) || !f.containsAirport(s2) || !f.hasRoute(s1, s2, processed2))
                    cout << "THE INPUTS ARE INVALID" << endl;
                else
                {
                    vector<string> str = f.get_Interchanges(f.Get_Minimum_Distance(s1, s2));
                    int len = str.size();
                    cout << "SOURCE AIRPORT : " << s1 << endl;
                    cout << "DESTINATION AIRPORT : " << s2 << endl;
                    cout << "DISTANCE : " << str[len - 1] << " KM" << endl;
                    cout << "NUMBER OF INTERCHANGES : " << str[len - 2] << endl;

                    cout << "~" << endl;
                    cout << "START  ==>  " << str[0] << endl;
                    for (int i = 1; i < len - 2; i++)
                    {
                        cout << str[i] << endl;
                    }
                    cout << str[len - 2] << "   ==>    END";
                    cout << "\n~" << endl;
                }
            }
            break;
        }
        else if (choice == 6)
        {
            printCodelist();
            cout << "\n1. TO ENTER SERIAL NO. OF AIRPORTS\n2. TO ENTER CODE OF AIRPORTS\n3. TO ENTER NAME OF AIRPORTS\n"
                 << endl;
            cout << "ENTER YOUR CHOICE:";
            int ch;
            cin >> ch;
            int j;
            string st1 = "", st2 = "";
            cout << "ENTER THE SOURCE AND DESTINATION AIRPORTS" << endl;
            if (ch == 1)
            {
                int a, b;
                cin >> a >> b;
                vector<string> keys;
                for (auto it = Flight_M::airports.begin(); it != Flight_M::airports.end(); it++)
                {
                    keys.push_back(it->first);
                }
                st1 = keys[a - 1];
                st2 = keys[b - 1];
            }
            else if (ch == 2)
            {
                string a, b;
                cin >> a >> b;
                for (auto it = Flight_M::airports.begin(); it != Flight_M::airports.end(); it++)
                {
                    string key = it->first;
                    string code = "";
                    int j = 0;
                    for (int k = 0; k < key.length(); k++)
                    {
                        char c = key[k];
                        if (isdigit(c))
                        {
                            code += c;
                            j++;
                        }
                        else if (isalpha(c))
                        {
                            code += c;
                        }
                    }
                    if (code.length() < 2)
                        code += toupper(key[1]);
                    if (code == a)
                    {
                        st1 = key;
                    }
                    if (code == b)
                    {
                        st2 = key;
                    }
                }
            }
            else if (ch == 3)
            {
                cin >> st1 >> st2;
            }
            else
            {
                cout << "Invalid choice" << endl;
                break;
            }
            unordered_map<string, bool> processed;
            if (!f.containsAirport(st1) || !f.containsAirport(st2) || !f.hasRoute(st1, st2, processed))
                cout << "THE INPUTS ARE INVALID" << endl;
            int op = f.dijkstra(st1, st2, false);
            if (op > 20)
            {
                cout << "FARE PRICE TO TRAVEL FROM " << st1 << " TO " << st2 << " IS " << 50 << " DOLLARS" << endl;
            }
            else
            {
                cout << "FARE PRICE TO TRAVEL FROM " << st1 << " TO " << st2 << " IS " << op * 2.5 << " DOLLARS" << endl;
            }
            break;
        }
        else
        {
            cout << "Please enter a valid option!" << endl;
        }
    }
    return 0;
}