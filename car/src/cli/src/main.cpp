
#include <stdio.h>
#include <stdlib.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

static const char *DELIM = " ";
static const char *UNIX_DOMAIN_SOCKET_URI = "/tmp/.roscar.car.interface.soc";

bool process(const char *line);

int main(int argc, char **argv)
{
    printf("\nROS Car CLI\n\n");

    char *line = nullptr;
    bool ret = true;

    while (true)
    {
        if (!(line = readline("> ")))
        {
            // EOF
            break;
        }

        if (*line)
        {
            add_history(line);
            ret = process(line);
        }

        free(line);
        if (!ret)
        {
            // routine finished
            break;
        }
    }

    printf("\nROS Car CLI: Bye\n\n");
}

bool process(const char *line)
{
    vector<string> cmd;
    stringstream ss(line);
    string item;

    while (ss >> item)
    {
        cmd.push_back(item);
    }

    if (strcasecmp(cmd[0].c_str(), "info") == 0)
    {
        printf("Info command: %s\n", line);
    }

    return true;
}
