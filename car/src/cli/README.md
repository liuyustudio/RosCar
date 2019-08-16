# CLI

## Dependent library

- GUN - readline
  - Installation
    - Debian/Ubuntu

      ```sh
      sudo apt-get install libreadline-dev
      ```

    - centos/redhat/suse

      ```sh
      yum install readline-devel
      ```

## Compile & Link

- catkin_make

  ```sh
  roscd && cd .. && catkin_make
  ```

- g++

  ```sh
  g++ -o test test.cpp -lreadline
  ```
