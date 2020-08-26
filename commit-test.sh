#!/bin/bash

uname="testName"
email="email@test.com"

git filter-branch -f --env-filter "
GIT_AUTHOR_NAME=\"$uname\"
GIT_COMMITTER_NAME=\"$uname\"
GIT_AUTHOR_EMAIL=\"$email\"
GIT_COMMITTER_EMAIL=\"$email\"
" -- HEAD^..