# Contributing Guidelines
Whether it's a bug report, new feature, correction, or additional
documentation, we greatly value feedback and contributions from our community.

Please read through this document before submitting any issues or pull requests to ensure we have all the necessary
information to effectively respond to your bug report or contribution.


## Reporting Bugs/Feature Requests
We welcome you to use the GitHub issue tracker to report bugs or suggest features.

When filing an issue, please check [existing open][issues], or [recently closed][closed-issues], issues to make sure
 somebody else hasn't already reported the issue.
Please try to include as much information as you can. Details like these are incredibly useful:

* A reproducible test case or series of steps
* The version of our code being used
* Any modifications you've made relevant to the bug
* Anything unusual about your environment or deployment


## Contributing via Pull Requests
The following guidance should be up-to-date, but the documentation as found [here](https://ros-controls.github.io/control.ros.org/contributing.html#pull-requests) should prove as the final say.

Contributions via pull requests are much appreciated.
Before sending us a pull request, please ensure that:

1. Limited scope. Your PR should do one thing or one set of things. Avoid adding “random fixes” to PRs. Put those on separate PRs.
2. Give your PR a descriptive title. Add a short summary, if required.
3. Make sure the pipeline is green.
4. Don’t be afraid to request reviews from maintainers.
5. New code = new tests. If you are adding new functionality, always make sure to add some tests exercising the code and serving as live documentation of your original intention.

To send us a pull request, please:

1. Fork the repository.
2. Modify the source; please focus on the specific change you are contributing.
  If you also reformat all the code, it will be hard for us to focus on your change.
3. Ensure local tests pass. (`colcon test` and `pre-commit run` (requires you to install pre-commit by `pip3 install pre-commit`)
4. Commit to your fork using clear commit messages.
5. Send a pull request, answering any default questions in the pull request interface.
6. Pay attention to any automated CI failures reported in the pull request, and stay involved in the conversation.

GitHub provides additional documentation on [forking a repository](https://help.github.com/articles/fork-a-repo/) and
[creating a pull request](https://help.github.com/articles/creating-a-pull-request/).

Any contribution that you make to this repository will be under the 3-Clause BSD License, as dictated by that [license](https://opensource.org/licenses/BSD-3-Clause).


## Finding contributions to work on
Looking at the existing issues is a great way to find something to contribute on.
As this project, by default, uses the default GitHub issue labels
  (enhancement/bug/duplicate/help wanted/invalid/question/wontfix), looking at any ['help wanted'][help-wanted] issues
  is a great place to start.
