#include <umeyama_test/umeyama.hpp>

Umeyama::Umeyama() : Node("umeyama")
{

    std::cout << "start!\n";
    m_timer = this->create_wall_timer(
        500ms, std::bind(&Umeyama::timerCallback, this));
}

void Umeyama::timerCallback()
{
    std::cout << "timerCallback\n";
}