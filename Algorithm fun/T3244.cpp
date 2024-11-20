class Solution {
public:
    vector<int> shortestDistanceAfterQueries(int n, vector<vector<int>>& queries) {
        set<int> nums;
        for(int i=0;i<n;i++)
        {
            nums.insert(i);
        }
        vector<int> ans;
        for(int i=0;i<queries.size();i++)
        {

            nums.erase(nums.lower_bound(queries[i][0]+1),nums.upper_bound(queries[i][1]-1));
            ans.emplace_back(nums.size()-1);
        }
        return ans;
    }
};
