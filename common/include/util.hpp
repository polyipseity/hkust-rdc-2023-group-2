#pragma once

auto panic [[noreturn]] (char const message[] = "") -> void;
