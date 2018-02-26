#include <boost/test/unit_test.hpp>
#include <motors_elmo_ds402/Dummy.hpp>

using namespace motors_elmo_ds402;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    motors_elmo_ds402::DummyClass dummy;
    dummy.welcome();
}
